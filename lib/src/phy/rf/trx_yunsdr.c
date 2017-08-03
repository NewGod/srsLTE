/* 
 * yunsdr transceiver driver
 *
 * Copyright (C) 2012-2016 V3best
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>
#include <getopt.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>

#include "trx_driver.h"
#include "yunsdr_api.h"

#define __DEBUG__

#define DEFAULT_SAMPLES_COUNT 4096 

typedef struct {
    YUNSDR_DESCRIPTOR *dev;
    int sample_rate;
    int tx_channel_count;
    int rx_channel_count;
    int64_t rx_count;
    int64_t tx_count;
    char *device_arg;
    int16_t *rx_buffer;
    int16_t *tx_buffer;
    uint32_t tx_max_count;
} TRXYunsdrState;

static int64_t get_time_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

static void trx_yunsdr_end(TRXState *s1)
{
    TRXYunsdrState *s = s1->opaque;
    printf("rx_count: %" PRId64 "\n", s->rx_count);
    printf("tx_count: %" PRId64 "\n", s->tx_count);
    yunsdr_close_device(s->dev);
    free(s->rx_buffer);
    free(s->tx_buffer);
    free(s);
}

static inline int64_t ts_to_time(TRXYunsdrState *s, int64_t ts)
{
    int n, r;
    n = (ts / s->sample_rate);
    r = (ts % s->sample_rate);
    return (int64_t)n * 1000000 + (((int64_t)r * 1000000) / s->sample_rate);
}

static void trx_yunsdr_write(TRXState *s1, trx_timestamp_t timestamp, const void **samples, int count, int flags, int rf_port_index)
{
    int status;
    TRXYunsdrState *s = s1->opaque;
    int j;

    if (!(flags & TRX_WRITE_FLAG_PADDING)) {
        for(j = 0; j < s->tx_channel_count; j++) {
            float_to_int16(s->tx_buffer, samples[j], count * 2, 32767);
            status = yunsdr_write_submit(s->dev, (uint8_t *)s->tx_buffer, count * 4, j+1, (uint64_t)timestamp);
            if (status < 0) {
                printf("Failed to TX samples\n");
            }
        }
    }

    s->tx_count += count;
    //printf("tx_count = %u, timestamp = %lu\n", count, timestamp);
}

static int trx_yunsdr_read(TRXState *s1, trx_timestamp_t *ptimestamp, void **psamples, int count, int rf_port)
{
    int status;
    TRXYunsdrState *s = s1->opaque;
    float *tab1 = psamples[0];
    int j;

    uint64_t timestamp;

    for(j = 0; j < s->rx_channel_count; j++) {
        status = yunsdr_read_samples(s->dev, (uint8_t *)s->rx_buffer, count * 4, j+1, &timestamp);
        if (status < 0) {
            printf("Failed to read sample\n");
            return 0;
        }
        if(!j)
            *ptimestamp = timestamp;
        tab1 = psamples[j];

        int16_to_float(tab1, s->rx_buffer, count * 2, 1./32767.);
    }
    s->rx_count += count;
    //printf("rx_count = %u, timestamp = %lu\n", count, timestamp);

    return count;
}

/* This function can be used to automatically set the sample
   rate. Here we don't implement it, so the user has to force a given
   sample rate with the "sample_rate" configuration option */
static int trx_yunsdr_get_sample_rate(TRXState *s, TRXFraction *psample_rate,
        int *psample_rate_num, int sample_rate_min)
{
    return -1;
}


static void trx_set_tx_gain_func(TRXState *s1, double gain, int chain)
{
    int status;

    TRXYunsdrState *s = s1->opaque;

    if ((status = yunsdr_set_tx_attenuation(s->dev, TX1_CHANNEL, (90 - gain)*1000)) < 0) {
        fprintf(stderr,"Failed to set TX gain\n");
    }else
        printf("[yunsdr] set the TX gain to %f dB\n", gain);
}

static void trx_set_rx_gain_func(TRXState *s1, double gain, int chain)
{
    int status;

    TRXYunsdrState *s = s1->opaque;

    if ((status = yunsdr_set_rx_rf_gain(s->dev, RX1_CHANNEL, (uint32_t)gain)) < 0) {
        fprintf(stderr,"Failed to set RX gain\n");
    } else
        printf("[yunsdr] set RX gain to %f dB\n",gain);
}
/* Return the maximum number of samples per TX packet. Called by
   the application after trx_start_func. */
int trx_yunsdr_get_tx_samples_per_packet(TRXState *s)
{
    return DEFAULT_SAMPLES_COUNT;
}

static int trx_yunsdr_start(TRXState *s1, const TRXDriverParams *p)
{
    int status;
    TRXYunsdrState *s = s1->opaque;

    if (p->rf_port_count != 1)
        return -1; /* only one TX port is supported */

    s->sample_rate = p->sample_rate[0].num / p->sample_rate[0].den;
    s->tx_channel_count = p->tx_channel_count;
    s->rx_channel_count = p->rx_channel_count;

    status = posix_memalign((void **)&s->rx_buffer, 16, sizeof(int16_t) * s->tx_max_count * 2);
    if(status) {
        printf("Failed to alloc memory\n");
        return -1;
    }
    status = posix_memalign((void **)&s->tx_buffer, 16, sizeof(int16_t) * s->tx_max_count * 2);
    if(status) {
        printf("Failed to alloc memory\n");
        return -1;
    }

    // RX
    if ((status=yunsdr_set_rx_lo_freq(s->dev, (uint64_t)(p->rx_freq[0]))) < 0){
        fprintf(stderr,"Failed to set RX frequency\n");
    } else
        printf("[yunsdr] set RX frequency to %lu\n",(uint64_t)(p->rx_freq[0]));
    if ((status=yunsdr_set_rx_sampling_freq(s->dev, (uint32_t)(s->sample_rate))) < 0){
        fprintf(stderr,"Failed to set RX sample rate\n");
    }else
        printf("[yunsdr] set RX sample rate to %u\n", (uint32_t)(s->sample_rate));
    if ((status=yunsdr_set_rx_rf_bandwidth(s->dev, (uint32_t)(p->rx_bandwidth[0]))) < 0){
        fprintf(stderr,"Failed to set RX bandwidth\n");
    }else
        printf("[yunsdr] set RX bandwidth to %u\n",(uint32_t)(p->rx_bandwidth[0]));
    if ((status=yunsdr_set_rx_gain_control_mode(s->dev, RX1_CHANNEL, 0)) < 0){
        fprintf(stderr,"Failed to set RX Gain Control Mode\n");
    }else
        printf("[yunsdr] set RX Gain Control Mode MGC\n");

    if ((status=yunsdr_set_rx_rf_gain(s->dev, RX1_CHANNEL, (uint32_t)(p->rx_gain[0]))) < 0) {
        fprintf(stderr,"Failed to set RX gain\n");
    } else
        printf("[yunsdr] set RX gain to %u\n",(uint32_t)(p->rx_gain[0]));

    // TX
    if ((status=yunsdr_set_tx_lo_freq(s->dev, (uint64_t)p->tx_freq[0])) < 0){
        fprintf(stderr,"Failed to set TX frequency\n");
    }else
        printf("[yunsdr] set TX Frequency to %lu\n", (uint64_t)p->tx_freq[0]);

    if ((status=yunsdr_set_tx_sampling_freq(s->dev, (uint32_t)s->sample_rate)) < 0){
        fprintf(stderr,"Failed to set TX sample rate\n");
    }else
        printf("[yunsdr] set TX sampling rate to %u\n", (uint32_t)s->sample_rate);

    if ((status=yunsdr_set_tx_rf_bandwidth(s->dev, (uint32_t)p->tx_bandwidth[0])) <0){
        fprintf(stderr, "Failed to set TX bandwidth\n");
    }else
        printf("[yunsdr] set TX bandwidth to %u\n", (uint32_t)p->tx_bandwidth[0]);

    if ((status=yunsdr_set_tx_attenuation(s->dev, TX1_CHANNEL, (90 - (uint32_t)p->tx_gain[0])*1000)) < 0) {
        fprintf(stderr,"Failed to set TX gain\n");
    }else
        printf("[yunsdr] set the TX gain to %d\n", (uint32_t)p->tx_gain[0]);

    yunsdr_disable_timestamp(s->dev);
    if ((status = yunsdr_enable_rx(s->dev, DEFAULT_SAMPLES_COUNT*4 + 16, (s->rx_channel_count >= 2)?3:1, 1)) < 0) {
        fprintf(stderr,"Failed to enable RX module\n");
    }else
        printf("[yunsdr] RX module enabled \n");
    if ((status = yunsdr_enable_tx(s->dev, DEFAULT_SAMPLES_COUNT*4, 1, 1)) < 0) {
        fprintf(stderr,"Failed to enable TX module\n");
    }else
        printf("[yunsdr] TX module enabled \n");

    usleep(5000);
    yunsdr_enable_timestamp(s->dev);
    return 0;
}

int trx_driver_init(TRXState *s1)
{
    TRXYunsdrState *s;
    char *default_arg = "fpga0";

    if (s1->trx_api_version != TRX_API_VERSION) {
        fprintf(stderr, "ABI compatibility mismatch between LTEENB and TRX driver (LTEENB ABI version=%d, TRX driver ABI version=%d)\n",
                s1->trx_api_version, TRX_API_VERSION);
        return -1;
    }

    s = malloc(sizeof(TRXYunsdrState));
    memset(s, 0, sizeof(*s));
    s->device_arg = NULL;

    /* get device driver param */
    if ((s->device_arg = trx_get_param_string(s1, "arg")) == NULL) 
        s->device_arg = default_arg;

    if ((s->dev = yunsdr_open_device(0)) == NULL ) {
        fprintf(stderr,"Failed to open yunsdr device\n");
        free(s);
        return -1;
    }
    s->tx_max_count = DEFAULT_SAMPLES_COUNT;

    printf("[yunsdr] init dev ...\n");
    yunsdr_set_ref_clock (s->dev, INTERNAL_REFERENCE);
    yunsdr_set_vco_select (s->dev, AUXDAC1);
    yunsdr_set_auxdac (s->dev, 970);
    //yunsdr_set_adf4001 (s->dev, (26<<16)|10);
    yunsdr_set_duplex_select (s->dev, FDD);
    yunsdr_set_trx_select (s->dev, TX);

    printf("[yunsdr] rf io init end\n");
    s1->opaque = s;
    s1->trx_end_func = trx_yunsdr_end;
    s1->trx_write_func = trx_yunsdr_write;
    s1->trx_read_func = trx_yunsdr_read;
    s1->trx_start_func = trx_yunsdr_start;
    s1->trx_get_sample_rate_func = trx_yunsdr_get_sample_rate;
    s1->trx_set_tx_gain_func = trx_set_tx_gain_func;
    s1->trx_set_rx_gain_func = trx_set_rx_gain_func;
    s1->trx_get_tx_samples_per_packet_func = trx_yunsdr_get_tx_samples_per_packet;

    return 0;
}
