/*
 * yunsdr_api.c
 *
 *  Created on: 2015Äê9ÔÂ9ÈÕ
 *      Author: Eric
 */
#include <sched.h>
#include <sys/ioctl.h>

#include <math.h>
#include <assert.h>
#include <immintrin.h>

#include "yunsdr_api.h"

#define MAX_TX_BUFFSIZE (32*1024*1024)
#define MAX_RX_BUFFSIZE (32*1024*1024)

#define MIN(x, y)  ((x < y)?x:y)

struct trx_thread tx_priv;
void *thread_tx_func(void *arg);
#define MAX_RB_SIZE (2<<15)
struct ringbuffer {
    void *psamples[MAX_RB_SIZE];
    volatile uint32_t front;
    volatile uint32_t rear;
}tx_rb;


uint64_t yunsdr_ticksToTimeNs(const uint64_t ticks, const double rate)
{
    const long long ratell = (long long)(rate);
    const long long full = (long long)(ticks/ratell);
    const long long err = ticks - (full*ratell);
    const double part = full*(rate - ratell);
    const double frac = ((err - part)*1000000000)/rate;
    return (full*1000000000) + llround(frac);
}
uint64_t yunsdr_timeNsToTicks(const uint64_t timeNs, const double rate)
{
    const long long ratell = (long long)(rate);
    const long long full = (long long)(timeNs/1000000000);
    const long long err = timeNs - (full*1000000000);
    const double part = full*(rate - ratell);
    const double frac = part + ((err*rate)/1000000000);
    return (full*ratell) + llround(frac);
}
int32_t yunsdr_init_meta(YUNSDR_META **meta)
{
    int32_t ret;

    *meta = (YUNSDR_META *)calloc(1, MAX_TX_BUFFSIZE + sizeof(YUNSDR_META));
    if (!meta) {
        ret = -1;
    } else {
        ret = 0;
    }

    return ret;
}

void yunsdr_deinit_meta(YUNSDR_META *meta)
{
    free(meta);
}

YUNSDR_DESCRIPTOR *yunsdr_open_device(uint8_t id)
{
    int ret;
    YUNSDR_DESCRIPTOR *yunsdr;
    yunsdr = (YUNSDR_DESCRIPTOR *)calloc(1, sizeof(
                YUNSDR_DESCRIPTOR));
    if (!yunsdr) {
        //return (YUNSDR_DESCRIPTOR *)(-ENOMEM);
        return NULL;
    }

    if(yunsdr_init_meta(&yunsdr->tx_meta) < 0)
        goto err_init_tx_meta;
    if(yunsdr_init_meta(&yunsdr->rx_meta) < 0)
        goto err_init_rx_meta;

    yunsdr->fpga = fpga_open(id);

    if (yunsdr->fpga == NULL) {
        printf("Could not open FPGA %d\n", id);
        goto err_open_device;
    }
    yunsdr->id = id;

    // Reset
    fpga_reset(yunsdr->fpga);

    tx_rb.front = tx_rb.rear = 0;
    int i;
    for(i = 0; i < MAX_RB_SIZE; i++) {
        tx_rb.psamples[i] = NULL;
    }
    ret = pthread_create(&tx_priv.trx, NULL, thread_tx_func, yunsdr);
    if(ret != 0)
        perror("pthread_create");

    return yunsdr;

err_open_device:
    yunsdr_deinit_meta(yunsdr->rx_meta);
err_init_rx_meta:
    yunsdr_deinit_meta(yunsdr->tx_meta);
err_init_tx_meta:
    free(yunsdr);

    //return (YUNSDR_DESCRIPTOR *)(-EINVAL);
    return NULL;
}


int32_t yunsdr_close_device(YUNSDR_DESCRIPTOR *yunsdr)
{
    tx_priv.end = 0;
    pthread_cancel(tx_priv.trx);
    yunsdr_disable_timestamp(yunsdr);
    fpga_close(yunsdr->fpga);
    yunsdr_deinit_meta(yunsdr->tx_meta);
    yunsdr_deinit_meta(yunsdr->rx_meta);
    free(yunsdr);

    return 0;
}
/* Set the receive RF gain for the selected channel. */
int32_t yunsdr_set_rx_rf_gain (YUNSDR_DESCRIPTOR *yunsdr,
        RF_CHANNEL ch, int32_t gain_db)
{
    int ret;
    YUNSDR_CMD rx_gain;

    ret = 0;

    if(gain_db < 5 || gain_db > 70)
        return -EINVAL;

    (ch == RX1_CHANNEL)?
        (rx_gain.type = CMD_SET_RX_GAIN1):
        (rx_gain.type = CMD_SET_RX_GAIN2);
    rx_gain.value = gain_db;
    ret = fpga_send(yunsdr->fpga, 0, &rx_gain, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
    }

    return ret;
}

/* Set the RX RF bandwidth. */
int32_t yunsdr_set_rx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t bandwidth_hz)
{
    int ret;
    YUNSDR_CMD rx_bw;

    ret = 0;

    if(bandwidth_hz > 0 && bandwidth_hz < 60e6) {
        rx_bw.type = CMD_SET_RX_BW;
        rx_bw.value = bandwidth_hz;
        ret = fpga_send(yunsdr->fpga, 0, &rx_bw, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
    } else 
        ret = -EINVAL;

    return ret;
}

/* Set the RX sampling frequency. */
int32_t yunsdr_set_rx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t sampling_freq_hz)
{
    int ret;
    YUNSDR_CMD rx_sampling, sampling_1pps;

    ret = 0;

    if(sampling_freq_hz > 1e6 && sampling_freq_hz < 56e6) {
        rx_sampling.type = CMD_SET_RX_SAMPLING;
        rx_sampling.value = sampling_freq_hz;
        ret = fpga_send(yunsdr->fpga, 0, &rx_sampling, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
        sampling_1pps.type = CMD_SET_SAMPLING_PPS;
        sampling_1pps.value = sampling_freq_hz;
        ret = fpga_send(yunsdr->fpga, 0, &sampling_1pps, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
    } else 
        ret = -EINVAL;

    return ret;
}

/* Set the RX LO frequency. */
int32_t yunsdr_set_rx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint64_t lo_freq_hz)
{
    int ret;
    YUNSDR_CMD rx_freq;

    ret = 0;

    if(lo_freq_hz >= 70e6 && lo_freq_hz <= 6e9) {
        rx_freq.type = CMD_SET_RX_FREQ;
        rx_freq.value = lo_freq_hz >> 1;
        ret = fpga_send(yunsdr->fpga, 0, &rx_freq, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
    } else
        ret = -EINVAL;

    return ret;
}

/* Set the gain control mode for the selected channel. */
int32_t yunsdr_set_rx_gain_control_mode (YUNSDR_DESCRIPTOR *yunsdr,
        RF_CHANNEL ch, RF_GAIN_CTRL_MODE gc_mode)
{
    int ret;
    YUNSDR_CMD rx_gc;

    ret = 0;

    (ch == RX1_CHANNEL)?rx_gc.type = CMD_SET_RX_GC1:CMD_SET_RX_GC2;
    rx_gc.value = gc_mode;
    ret = fpga_send(yunsdr->fpga, 0, &rx_gc, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
    }

    return ret;
}

/* Set the transmit attenuation for the selected channel. */
int32_t yunsdr_set_tx_attenuation (YUNSDR_DESCRIPTOR *yunsdr,
        RF_CHANNEL ch, uint32_t attenuation_mdb)
{
    int ret;
    YUNSDR_CMD tx_atten;

    ret = 0;

    if(attenuation_mdb < 1000 || attenuation_mdb > 90000)
        return -EINVAL;

    (ch == TX1_CHANNEL)?
        tx_atten.type = CMD_SET_TX_ATTEN1:CMD_SET_TX_ATTEN2;
    tx_atten.value = attenuation_mdb;
    ret = fpga_send(yunsdr->fpga, 0, &tx_atten, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
    }

    return ret;
}

/* Set the TX RF bandwidth. */
int32_t yunsdr_set_tx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t  bandwidth_hz)
{
    int ret;
    YUNSDR_CMD tx_bw;

    ret = 0;

    if(bandwidth_hz > 0 && bandwidth_hz < 60e6) {
        tx_bw.type = CMD_SET_TX_BW;
        tx_bw.value = bandwidth_hz;
        ret = fpga_send(yunsdr->fpga, 0, &tx_bw, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
    } else
        ret = -EINVAL;

    return ret;
}


/* Set the TX sampling frequency. */
int32_t yunsdr_set_tx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t sampling_freq_hz)
{
    int ret;
    YUNSDR_CMD tx_sampling, sampling_1pps;

    ret = 0;

    if(sampling_freq_hz > 1e6 && sampling_freq_hz < 56e6) {
        tx_sampling.type = CMD_SET_TX_SAMPLING;
        tx_sampling.value = sampling_freq_hz;
        ret = fpga_send(yunsdr->fpga, 0, &tx_sampling, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
        sampling_1pps.type = CMD_SET_SAMPLING_PPS;
        sampling_1pps.value = sampling_freq_hz;
        ret = fpga_send(yunsdr->fpga, 0, &sampling_1pps, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
    } else 
        ret = -EINVAL;

    return ret;
}

/* Set the TX LO frequency. */
int32_t yunsdr_set_tx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint64_t lo_freq_hz)
{
    int ret;
    YUNSDR_CMD tx_freq;

    ret = 0;

    if(lo_freq_hz >= 70e6 && lo_freq_hz <= 6e9) {
        tx_freq.type = CMD_SET_TX_FREQ;
        tx_freq.value = lo_freq_hz >> 1;
        ret = fpga_send(yunsdr->fpga, 0, &tx_freq, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
        if(ret < 0){
            printf("%s failed\n", __func__);
            ret = -EIO;
        }
    } else 
        ret = -EINVAL;

    return ret;
}

/* Get current receive RF gain for the selected channel. */
int32_t yunsdr_get_rx_rf_gain (YUNSDR_DESCRIPTOR *yunsdr, RF_CHANNEL ch, int32_t *gain_db)
{
    return -1;
}

/* Get the RX RF bandwidth. */
int32_t yunsdr_get_rx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *bandwidth_hz)
{
    return -1;
}

/* Get current RX sampling frequency. */
int32_t yunsdr_get_rx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *sampling_freq_hz)
{
    return -1;
}

/* Get current RX LO frequency. */
int32_t yunsdr_get_rx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr, uint64_t *lo_freq_hz)
{
    return -1;
}

/* Get the gain control mode for the selected channel. */
int32_t yunsdr_get_rx_gain_control_mode (YUNSDR_DESCRIPTOR *yunsdr, RF_CHANNEL ch, uint8_t *gc_mode)
{
    return -1;
}

/* Get current transmit attenuation for the selected channel. */
int32_t yunsdr_get_tx_attenuation (YUNSDR_DESCRIPTOR *yunsdr, RF_CHANNEL ch, uint32_t *attenuation_mdb)
{
    return -1;
}

/* Get the TX RF bandwidth. */
int32_t yunsdr_get_tx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *bandwidth_hz)
{
    return -1;
}

/* Get current TX sampling frequency. */
int32_t yunsdr_get_tx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *sampling_freq_hz)
{
    return -1;
}

/* Get current TX LO frequency. */
int32_t yunsdr_get_tx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr, uint64_t *lo_freq_hz)
{
    return -1;
}

int32_t yunsdr_set_tone_bist (YUNSDR_DESCRIPTOR *yunsdr, uint8_t enable)
{
    return -1;
}

int32_t yunsdr_set_ref_clock (YUNSDR_DESCRIPTOR *yunsdr,
        REF_SELECT select)
{
    int32_t ret;
    YUNSDR_CMD clk_ref;

    clk_ref.type = CMD_SET_CLK_REF;
    clk_ref.value = select;

    ret = fpga_send(yunsdr->fpga, 0, &clk_ref, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    return 0;
}
int32_t yunsdr_set_vco_select (YUNSDR_DESCRIPTOR *yunsdr,
        VCO_CAL_SELECT select)
{
    int32_t ret;
    YUNSDR_CMD vco_ref;

    vco_ref.type = CMD_SET_VCO_REF;
    vco_ref.value = select;

    ret = fpga_send(yunsdr->fpga, 0, &vco_ref, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    return 0;
}
int32_t yunsdr_set_trx_select (YUNSDR_DESCRIPTOR *yunsdr,
        TRX_SWITCH select)
{
    int32_t ret;
    YUNSDR_CMD trx_switch;

    trx_switch.type = CMD_SET_TRX_SWITCH;
    trx_switch.value = select;

    ret = fpga_send(yunsdr->fpga, 0, &trx_switch, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    return 0;
}
int32_t yunsdr_set_duplex_select (YUNSDR_DESCRIPTOR *yunsdr,
        DUPLEX_SELECT select)
{
    int32_t ret;
    YUNSDR_CMD duplex;

    duplex.type = CMD_SET_DUPLEX;
    duplex.value = select;

    ret = fpga_send(yunsdr->fpga, 0, &duplex, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }
    return 0;
}

int32_t yunsdr_set_adf4001 (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t val)
{
    int32_t ret;
    YUNSDR_CMD adf4001;

    adf4001.type = CMD_SET_ADF4001;
    adf4001.value = val;

    ret = fpga_send(yunsdr->fpga, 0, &adf4001, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    //((Ncount << 16) | Rcount)

    return 0;
}

int32_t yunsdr_set_auxdac (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t mV)
{
    int32_t ret;
    YUNSDR_CMD auxdac1;

    auxdac1.type = CMD_SET_AUXDAC1;
    auxdac1.value = mV;

    ret = fpga_send(yunsdr->fpga, 0, &auxdac1, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    return 0;
}
int32_t yunsdr_enable_timestamp (YUNSDR_DESCRIPTOR *yunsdr)
{
    int32_t ret;
    YUNSDR_CMD timestamp_en;

    timestamp_en.type = CMD_SET_MTIME_START;
    timestamp_en.value = 1;

    ret = fpga_send(yunsdr->fpga, 0, &timestamp_en, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    return 0;
}
int32_t yunsdr_disable_timestamp (YUNSDR_DESCRIPTOR *yunsdr)
{
    int32_t ret;
    YUNSDR_CMD timestamp_en;

    timestamp_en.type = CMD_SET_MTIME_START;
    timestamp_en.value = 0;

    ret = fpga_send(yunsdr->fpga, 0, &timestamp_en, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    return 0;
}
int32_t yunsdr_read_timestamp (YUNSDR_DESCRIPTOR *yunsdr,
        uint64_t *timestamp)
{
    return 0;
}

int32_t yunsdr_enable_rx(YUNSDR_DESCRIPTOR *yunsdr, uint32_t nbyte_per_packet,
        uint8_t ch, uint8_t enable)
{
    int32_t ret;
    YUNSDR_CMD rx_burst, rx_chan;

    rx_burst.type = CMD_SET_RX_BURST_LEN;
    rx_burst.value = nbyte_per_packet/4;

    ret = fpga_send(yunsdr->fpga, 0, &rx_burst, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    rx_chan.type = CMD_SET_RX_CHANNEL;
    rx_chan.value = ch;

    ret = fpga_send(yunsdr->fpga, 0, &rx_chan, sizeof(YUNSDR_CMD)/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }


    return 0;
}
int32_t yunsdr_enable_tx(YUNSDR_DESCRIPTOR *yunsdr, uint32_t nbyte_per_packet,
        uint8_t ch, uint8_t enable)
{
    return 0;
}

int32_t yunsdr_read_samples(YUNSDR_DESCRIPTOR *yunsdr,
        uint8_t *buffer, uint32_t nbyte, uint8_t ch, uint64_t *timestamp)
{
    int ret;

    if(nbyte > MAX_RX_BUFFSIZE)
        return -EINVAL;

    ret = fpga_recv(yunsdr->fpga, ch, yunsdr->rx_meta, 
            (nbyte + sizeof(YUNSDR_META))/4, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }

    *timestamp = ((uint64_t)yunsdr->rx_meta->timestamp_h)<<32 | 
        yunsdr->rx_meta->timestamp_l;
    memcpy(buffer, (unsigned char *)yunsdr->rx_meta->payload, nbyte);

    return nbyte;
}

int32_t yunsdr_write_samples(YUNSDR_DESCRIPTOR *yunsdr,
        uint8_t *buffer, uint32_t nbyte, uint8_t ch,uint64_t timestamp)
{
    int ret;
    uint8_t timestamp_en;

    if(nbyte > MAX_TX_BUFFSIZE)
        return -EINVAL;

    if(timestamp > 0)
        timestamp_en = 1;
    else
        timestamp_en = 0;

    yunsdr->tx_meta->timestamp_l = (uint32_t)timestamp;
    yunsdr->tx_meta->timestamp_h = (uint32_t)(timestamp>>32);

    yunsdr->tx_meta->head = (ch<<16) | (timestamp_en<<19) | 0xEB90;

    yunsdr->tx_meta->nsamples = nbyte / 4;

    memcpy(yunsdr->tx_meta->payload, buffer, nbyte);

    ret = fpga_send(yunsdr->fpga, ch, yunsdr->tx_meta, 
            (nbyte + sizeof(YUNSDR_META))/4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
    }

    //debug(DEBUG_WARN, "fpga_send = %u, nsample = %u\n", ret, yunsdr->tx_meta->nsamples);
    return nbyte;
}

int32_t yunsdr_write_submit(YUNSDR_DESCRIPTOR *yunsdr,
        uint8_t *buffer, uint32_t nbyte, uint8_t ch, uint64_t timestamp)
{
    uint8_t timestamp_en;
    YUNSDR_META *tx_meta;
    uint32_t dma_len;

    dma_len = (nbyte + sizeof(YUNSDR_META)) / 4; //sizeof word
    if(dma_len % 16 != 0) {
        dma_len += (16 - dma_len%16);
    }

    void *psamples = calloc(1, dma_len * 4);
    tx_meta = (YUNSDR_META *)psamples;
    if(timestamp > 0)
        timestamp_en = 1;
    else
        timestamp_en = 0;

    tx_meta->timestamp_l = (uint32_t)timestamp;
    tx_meta->timestamp_h = (uint32_t)(timestamp>>32);

    tx_meta->head = (ch<<16) | (timestamp_en<<19) | 0xEB90;

    tx_meta->nsamples = nbyte / 4;

    memcpy(tx_meta->payload, buffer, nbyte);

    if((tx_rb.rear + 1) % MAX_RB_SIZE != tx_rb.front) {
        tx_rb.psamples[tx_rb.rear] = psamples;
        tx_rb.rear = (tx_rb.rear + 1) % MAX_RB_SIZE;
    } else
        printf("full ...\n");

    return nbyte;
}

static int32_t yunsdr_write_stream(YUNSDR_DESCRIPTOR *yunsdr, void *buffer)
{
    int ret;
    uint32_t ch;
    uint32_t nbyte;
    YUNSDR_META *tx_meta;

    tx_meta = (YUNSDR_META *)buffer;
    ch = (tx_meta->head >> 16)&0x3;

    nbyte = tx_meta->nsamples * 4;

    //printf("%s:%d\n", __func__, __LINE__);
    ret = fpga_send(yunsdr->fpga, ch, buffer, 
            (nbyte + sizeof(YUNSDR_META)) / 4, 0, 1, 25000);
    if(ret < 0){
        printf("%s failed\n", __func__);
        ret = -EIO;
        return ret;
    }
    //printf("%s:%d\n", __func__, __LINE__);
    return nbyte;
}

void *thread_tx_func(void *arg)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(4, &set);
    if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &set) < 0)
        perror("pthread_setaffinity_np");
    sem_init(&tx_priv.start, 0, 0);
    sem_init(&tx_priv.finish, 0, 0);
    tx_priv.end = 1;

    while(tx_priv.end) {
        if(tx_rb.front == tx_rb.rear) { 
            // sem_wait(&tx_priv.start);
            sleep(0);
        } else {
            if(tx_rb.psamples[tx_rb.front] != NULL) {
                yunsdr_write_stream((YUNSDR_DESCRIPTOR *)arg,
                        tx_rb.psamples[tx_rb.front]);
                free(tx_rb.psamples[tx_rb.front]);
                tx_rb.psamples[tx_rb.front] = NULL;
            }
            //sem_post(&tx_priv.finish);
            tx_rb.front = (tx_rb.front + 1) % MAX_RB_SIZE;
        }
    }

    return NULL;
}


#ifdef __DEBUG__
void debug(int level, const char *fmt, ...)
{
    if(level <= DEBUG_OUTPUT_LEVEL)
    {
        va_list ap;
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
    }
}
#else
void debug(int level, const char *fmt, ...)
{
}
#endif

/* Note: src and dst must be 16 byte aligned */
void float_to_int16(int16_t *dst, const float *src, int n, float mult)
{
    const __m128 *p;
    __m128i *q, a0, a1;
    __m128 mult1;

    mult1 = _mm_set1_ps(mult);
    p = (const void *)src;
    q = (void *)dst;

    while (n >= 16) {
        a0 = _mm_cvtps_epi32(p[0] * mult1);
        a1 = _mm_cvtps_epi32(p[1] * mult1);
        q[0] = _mm_packs_epi32(a0, a1);
        a0 = _mm_cvtps_epi32(p[2] * mult1);
        a1 = _mm_cvtps_epi32(p[3] * mult1);
        q[1] = _mm_packs_epi32(a0, a1);
        p += 4;
        q += 2;
        n -= 16;
    }
    if (n >= 8) {
        a0 = _mm_cvtps_epi32(p[0] * mult1);
        a1 = _mm_cvtps_epi32(p[1] * mult1);
        q[0] = _mm_packs_epi32(a0, a1);
        p += 2;
        q += 1;
        n -= 8;
    }
    if (n != 0) {
        /* remaining samples (n <= 7) */
        do {
            a0 = _mm_cvtps_epi32(_mm_load_ss((float *)p) * mult);
            *(int16_t *)q = _mm_cvtsi128_si32 (_mm_packs_epi32(a0, a0));
            p = (__m128 *)((float *)p + 1);
            q = (__m128i *)((int16_t *)q + 1);
            n--;
        } while (n != 0);
    }
}
/* Note: src and dst must be 16 byte aligned */
void int16_to_float(float *dst, const int16_t *src, int len, float mult)
{
    __m128i a0, a1, a, b, sign;
    __m128 mult1;

    mult1 = _mm_set1_ps(mult);
    while (len >= 8) {
        a = *(__m128i *)&src[0];
        a0 = _mm_cvtepi16_epi32(a);
        b  = _mm_srli_si128(a, 8);
        a1 = _mm_cvtepi16_epi32(b);
        *(__m128 *)&dst[0] = _mm_cvtepi32_ps(a0) * mult1;
        *(__m128 *)&dst[4] = _mm_cvtepi32_ps(a1) * mult1;
        dst += 8;
        src += 8;
        len -= 8;
    }
    /* remaining data */
    while (len != 0) {
        _mm_store_ss(&dst[0], _mm_cvtsi32_ss(_mm_setzero_ps(), src[0]) * mult1);
        dst++;
        src++;
        len--;
    }
}
