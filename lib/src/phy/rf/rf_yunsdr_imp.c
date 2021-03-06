/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */


#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "srslte/srslte.h"
#include "rf_yunsdr_imp.h"
#include "srslte/phy/rf/rf.h"

#include "yunsdr_api.h"

#define DEFAULT_SAMPLES_COUNT (5760*2)
#define CONVERT_BUFFER_SIZE 240*1024

typedef struct {
    YUNSDR_DESCRIPTOR *dev;
    double sample_rate;
    int16_t rx_buffer[CONVERT_BUFFER_SIZE]; 
    int16_t tx_buffer[CONVERT_BUFFER_SIZE]; 
    uint32_t samples_per_packet;
    uint8_t timestamp_en;

} rf_yunsdr_handler_t;


int yunsdr_error(void *h)
{
    return 0;
}

void rf_yunsdr_get_freq_range(void *h)
{

}

void rf_yunsdr_suppress_handler(const char *x)
{
    // not supported
}

void rf_yunsdr_msg_handler(const char *msg)
{
    // not supported
}

void rf_yunsdr_suppress_stdout(void *h)
{
    // not supported
}

void rf_yunsdr_register_error_handler(void *notused, srslte_rf_error_handler_t new_handler)
{
    // not supported
}

char* rf_yunsdr_devname(void* h)
{
    return "yunsdr";
}

bool rf_yunsdr_rx_wait_lo_locked(void *h)
{
    usleep(1000);
    printf("TODO: implement rf_yunsdr_rx_wait_lo_locked()\n");
    return true;
}

int rf_yunsdr_start_rx_stream(void *h, bool now)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;

    yunsdr_disable_timestamp(handler->dev);
    if (yunsdr_enable_rx(handler->dev, handler->samples_per_packet*4 + 16, 1, 1) < 0) {
        fprintf(stderr,"Failed to enable RX module\n");
        return SRSLTE_ERROR;
    }
#if 0
    if (yunsdr_enable_tx(handler->dev, handler->samples_per_packet*4, 1, 1) < 0) {
        fprintf(stderr,"Failed to enable TX module\n");
    }else
        printf("[yunsdr] TX module enabled \n");
#endif
    usleep(5000);
    yunsdr_enable_timestamp(handler->dev);
    handler->timestamp_en = 1;

    return SRSLTE_SUCCESS;
}

int rf_yunsdr_start_tx_stream(void *h)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
#if 0
    if (yunsdr_enable_tx(handler->dev, handler->samples_per_packet*4, 1, 1) < 0) {
        fprintf(stderr,"Failed to enable TX module\n");
    }else
        printf("[yunsdr] TX module enabled \n");
#endif
    return SRSLTE_SUCCESS;
}

int rf_yunsdr_stop_rx_stream(void *h)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
#if 0
    yunsdr_enable_tx(handler->dev, 0, 1, 0);
#endif
    return SRSLTE_SUCCESS;
}


int rf_yunsdr_stop_tx_stream(void *h)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    return SRSLTE_SUCCESS;
}

void rf_yunsdr_flush_buffer(void *h)
{

}

bool rf_yunsdr_has_rssi(void *h)
{
    printf("TODO: implement rf_yunsdr_has_rssi()\n");
    return false;
}

float rf_yunsdr_get_rssi(void *h)
{
    printf("TODO: implement rf_yunsdr_get_rssi()\n");
    return 0.0;
}

//TODO: add multi-channel support
int rf_yunsdr_open_multi(char *args, void **h, uint32_t nof_channels)
{
    // create handler
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) malloc(sizeof(rf_yunsdr_handler_t));
    if (!handler) {
        perror("malloc");
        return SRSLTE_ERROR; 
    }

    bzero(handler, sizeof(rf_yunsdr_handler_t));
    *h = handler;
    if ((handler->dev = yunsdr_open_device(0)) == NULL ) {
        fprintf(stderr,"Failed to open yunsdr device\n");
        free(handler);
        return SRSLTE_ERROR;
    }
    handler->timestamp_en = 0;
    printf("[yunsdr] init dev ...\n");
    yunsdr_set_ref_clock (handler->dev, INTERNAL_REFERENCE);
    yunsdr_set_vco_select (handler->dev, AUXDAC1);
    yunsdr_set_auxdac (handler->dev, 1600);
    //yunsdr_set_adf4001 (handler->dev, (26<<16)|10);
    yunsdr_set_duplex_select (handler->dev, FDD);
    yunsdr_set_trx_select (handler->dev, 0);
    if (yunsdr_set_rx_gain_control_mode(handler->dev, RX1_CHANNEL, 0) < 0){
        fprintf(stderr,"Failed to set RX Gain Control Mode\n");
    }else
        printf("[yunsdr] set RX Gain Control Mode MGC\n");

    return SRSLTE_SUCCESS;
}

int rf_yunsdr_open(char *args, void **h)
{
    return rf_yunsdr_open_multi(args, h, 1);
}

int rf_yunsdr_close(void *h)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    yunsdr_disable_timestamp(handler->dev);
    yunsdr_close_device(handler->dev);
    free(handler);

    return SRSLTE_SUCCESS;
}

void rf_yunsdr_set_master_clock_rate(void *h, double rate)
{
    // Allow the yunsdr to automatically set the appropriate clock rate
    // TODO: implement this function
}


bool rf_yunsdr_is_master_clock_dynamic(void *h)
{
    printf("TODO: implement rf_yunsdr_is_master_clock_dynamic()\n");
    return false;
}

double rf_yunsdr_set_rx_srate(void *h, double rate)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;

    if(rate < 2000000)
        rate = 3840000;
    if (yunsdr_set_rx_sampling_freq(handler->dev, (uint32_t)rate) < 0){
        fprintf(stderr,"Failed to set RX sample rate\n");
        return SRSLTE_ERROR;
    }else
        printf("[yunsdr] set RX sample rate to %u\n", (uint32_t)rate);
    handler->sample_rate = rate;
    if (yunsdr_set_rx_rf_bandwidth(handler->dev, (uint32_t)(rate/1.5)) < 0){
        fprintf(stderr,"Failed to set RX bandwidth\n");
    }else
        printf("[yunsdr] set RX bandwidth to %u\n",(uint32_t)(rate/1.5));

    handler->samples_per_packet = (int)handler->sample_rate/1000;

    return rate;
}

double rf_yunsdr_set_tx_srate(void *h, double rate)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    if (yunsdr_set_tx_sampling_freq(handler->dev, (uint32_t)rate) < 0){
        fprintf(stderr,"Failed to set TX sample rate\n");
        return SRSLTE_ERROR;
    }else
        printf("[yunsdr] set TX sample rate to %u\n", (uint32_t)rate);
    handler->sample_rate = rate;
    if (yunsdr_set_tx_rf_bandwidth(handler->dev, (uint32_t)(rate/1.5)) < 0){
        fprintf(stderr,"Failed to set TX bandwidth\n");
    }else
        printf("[yunsdr] set TX bandwidth to %u\n",(uint32_t)(rate/1.5));

    return rate;
}

double rf_yunsdr_set_rx_gain(void *h, double gain)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    if (yunsdr_set_rx_rf_gain(handler->dev, RX1_CHANNEL, (uint32_t)gain) < 0) {
        return SRSLTE_ERROR;
        fprintf(stderr,"Failed to set RX gain\n");
    } else
        printf("[yunsdr] set RX gain to %u\n",(uint32_t)gain);

    return gain;
}


double rf_yunsdr_set_tx_gain(void *h, double gain)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;

    if (yunsdr_set_tx_attenuation(handler->dev, TX1_CHANNEL, (90 - (uint32_t)gain)*1000) < 0) {
        fprintf(stderr,"Failed to set TX gain\n");
        return SRSLTE_ERROR;
    }else
        printf("[yunsdr] set the TX gain to %d\n", (uint32_t)gain);

    return gain;
}


double rf_yunsdr_get_rx_gain(void *h)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    return SRSLTE_ERROR;
}


double rf_yunsdr_get_tx_gain(void *h)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    return SRSLTE_ERROR;
}


double rf_yunsdr_set_rx_freq(void *h, double freq)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    if (yunsdr_set_rx_lo_freq(handler->dev, (uint64_t)freq) < 0){
        fprintf(stderr,"Failed to set RX frequency\n");
        return SRSLTE_ERROR;
    } else
        printf("[yunsdr] set RX frequency to %lu\n",(uint64_t)freq);

    return freq;
}

double rf_yunsdr_set_tx_freq(void *h, double freq)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    if (yunsdr_set_tx_lo_freq(handler->dev, (uint64_t)freq) < 0){
        fprintf(stderr,"Failed to set TX frequency\n");
        return SRSLTE_ERROR;
    } else
        printf("[yunsdr] set TX frequency to %lu\n",(uint64_t)freq);

    return freq;
}


void rf_yunsdr_get_time(void *h, time_t *secs, double *frac_secs) 
{

}

static void timestamp_to_secs(uint32_t rate, uint64_t timestamp, time_t *secs, double *frac_secs)
{
    double totalsecs = (double) timestamp/rate;
    time_t secs_i = (time_t) totalsecs;
    if (secs) {
        *secs = secs_i;
    }
    if (frac_secs) {
        *frac_secs = totalsecs-secs_i;
    }
}

static void secs_to_timestamps(uint32_t rate, time_t secs, double frac_secs, uint64_t *timestamp)
{
    double totalsecs = (double) secs + frac_secs;
    if (timestamp) {
        *timestamp = rate * totalsecs;
    }
}

//TODO: add multi-channel support
int  rf_yunsdr_recv_with_time_multi(void *h,
        void **data,
        uint32_t nsamples,
        bool blocking,
        time_t *secs,
        double *frac_secs)
{
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    float *tab1 = (float *)data[0];
    uint64_t timestamp;
    YUNSDR_META *rx_meta = handler->dev->rx_meta;
    int ret = 0;
    int n = 0;
    void *psamples = (void *)handler->rx_buffer;

    if (2*nsamples > CONVERT_BUFFER_SIZE) {
        fprintf(stderr, "RX failed: nsamples exceeds buffer size (%d>%d)\n", nsamples, CONVERT_BUFFER_SIZE);
        return -1;
    }
#if 0
    if(yunsdr_read_samples(handler->dev, (uint8_t *)handler->rx_buffer, nsamples * 4, 1, &timestamp) < 0) {
        printf("Failed to read sample\n");
        return SRSLTE_ERROR;
    }
#else
    size_t remaining = nsamples;
        do {
             size_t rx_samples;
             if(remaining > handler->samples_per_packet)
                 rx_samples = handler->samples_per_packet;
             else
                 rx_samples = remaining;
             ret = yunsdr_read_samples(handler->dev, psamples, rx_samples*4, 1, &timestamp);
             if(ret < 0)
                 return SRSLTE_ERROR;
             if(!n) {
                 timestamp_to_secs(handler->sample_rate, timestamp, secs, frac_secs);
             }
             n = n + rx_samples;
             psamples = psamples + (n * 4);
             remaining -= rx_samples;
        } while (remaining > 0);

        if (n != nsamples) {
            fprintf(stderr, "Couldn't read all samples.\n");
            return SRSLTE_ERROR;
        }
#endif
    //printf("rx_nsamples:%u, timestamp: %llu\n", nsamples, timestamp);
    //timestamp_to_secs(handler->sample_rate, timestamp, secs, frac_secs);
    srslte_vec_convert_if(handler->rx_buffer, 16384.0, (float*)data[0], 2*nsamples);
    //printf("RX: sec:%u, frac_secs:%f\n", *(uint64_t *)secs, *frac_secs);

    return nsamples;
}

int rf_yunsdr_recv_with_time(void *h,
        void *data,
        uint32_t nsamples,
        bool blocking,
        time_t *secs,
        double *frac_secs)
{
    return rf_yunsdr_recv_with_time_multi(h, &data, nsamples, blocking, secs, frac_secs);
}

int rf_yunsdr_send_timed_multi(void *h,
                     void *data[4],
                     int nsamples,
                     time_t secs,
                     double frac_secs,
                     bool has_time_spec,
                     bool blocking,
                     bool is_start_of_burst,
                     bool is_end_of_burst)
{
  return rf_yunsdr_send_timed(h, data[0], nsamples, secs, frac_secs, has_time_spec, blocking, is_start_of_burst, is_end_of_burst);
}


int rf_yunsdr_send_timed(void *h,
        void *data,
        int nsamples,
        time_t secs,
        double frac_secs,
        bool has_time_spec,
        bool blocking,
        bool is_start_of_burst,
        bool is_end_of_burst)
{
    int flags;
    uint64_t timestamp;
    rf_yunsdr_handler_t *handler = (rf_yunsdr_handler_t*) h;
    YUNSDR_META *tx_meta = handler->dev->tx_meta;
    int ret = 0;
    int n = 0;
    void *psamples = (void *)handler->tx_buffer;

    if (2*nsamples > CONVERT_BUFFER_SIZE) {
        fprintf(stderr, "TX failed: nsamples exceeds buffer size (%d>%d)\n", nsamples, CONVERT_BUFFER_SIZE);
        return -1;
    }

    srslte_vec_convert_fi(data, (float)16384., handler->tx_buffer, 2*nsamples);
    //srslte_vec_convert_fi(data, (int16_t *)tx_meta->payload, 16384., 2*nsamples);
    secs_to_timestamps(handler->sample_rate, secs, frac_secs, &timestamp);

    //printf("TX: sec:%u, frac_secs:%f\n", (uint64_t)secs, frac_secs);
    //printf("tx_nsamples:%u, timestamp:%llu\n", nsamples, timestamp);
    if(handler->timestamp_en) {
        //int ret = yunsdr_write_submit(handler->dev, (uint8_t *)handler->tx_buffer, nsamples*4, 1, (uint64_t)timestamp);
        //int ret = yunsdr_write_samples(handler->dev, NULL, nsamples*4, 1, (uint64_t)timestamp);
        //if(ret < 0)
            //return SRSLTE_ERROR;
        size_t remaining = nsamples;
        do {
             size_t tx_samples;
             if(remaining > DEFAULT_SAMPLES_COUNT)
                 tx_samples = DEFAULT_SAMPLES_COUNT;
             else
                 tx_samples = remaining;
             ret = yunsdr_write_samples(handler->dev, psamples, tx_samples*4, 1, (uint64_t)timestamp);
             if(ret < 0)
                 return SRSLTE_ERROR;
             n = n + tx_samples;
             psamples = psamples + (n * 4);
             timestamp += tx_samples;
             remaining -= tx_samples;
        } while (remaining > 0);

        if (n != nsamples) {
            fprintf(stderr, "Couldn't write all samples.\n");
            return SRSLTE_ERROR;
        }
    } else {
        fprintf(stderr, "TX failed: timestamp in hardware is not enabled;\n");
        return SRSLTE_ERROR;
    }

    return nsamples;
}

srslte_rf_info_t * rf_yunsdr_get_info(void* h){
    printf("TODO: implement rf_yunsdr_get_info()\n");
	return NULL;
}
