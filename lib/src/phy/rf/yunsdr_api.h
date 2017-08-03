/*
 * yunsdr_api.h
 *
 *  Created on: 2016/5/9
 *      Author: Eric
 */
#ifndef __YUNSDR_API_H__
#define __YUNSDR_API_H__
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>

#include <riffa.h>
#include "regmap.h"

struct trx_thread {
    pthread_t trx;
    sem_t start;
    sem_t finish;
    int end;
};

typedef struct yunsdr_meta {
    uint32_t head;
    uint32_t nsamples;
    uint32_t timestamp_l;
    uint32_t timestamp_h;
    uint32_t payload[0];
}YUNSDR_META;

typedef struct yunsdr_device_descriptor {
    fpga_t *fpga;
    int32_t id;
    int32_t status;
    YUNSDR_META *tx_meta;
    YUNSDR_META *rx_meta;
}YUNSDR_DESCRIPTOR;

typedef enum {
    TX1_CHANNEL,
    TX2_CHANNEL,
    RX1_CHANNEL,
    RX2_CHANNEL,
}RF_CHANNEL;

typedef enum rf_gain_ctrl_mode {
    RF_GAIN_MGC,
    RF_GAIN_FASTATTACK_AGC,
    RF_GAIN_SLOWATTACK_AGC,
}RF_GAIN_CTRL_MODE;

typedef enum ref_select {
    INTERNAL_REFERENCE = 0,
    EXTERNAL_REFERENCE,
}REF_SELECT;

typedef enum vco_cal_select {
    ADF4001 = 0,
    AUXDAC1,
}VCO_CAL_SELECT;

typedef enum duplex_select {
    TDD = 0,
    FDD,
}DUPLEX_SELECT;

typedef enum trx_switch {
    RX = 0,
    TX,
}TRX_SWITCH;


extern uint64_t yunsdr_ticksToTimeNs(const uint64_t ticks, const double rate);

extern uint64_t yunsdr_timeNsToTicks(const uint64_t timeNs, const double rate);

extern YUNSDR_DESCRIPTOR *yunsdr_open_device(uint8_t id);

extern int32_t yunsdr_close_device(YUNSDR_DESCRIPTOR *yunsdr);

/* Get current receive RF gain for the selected channel. */
extern int32_t yunsdr_get_rx_rf_gain (YUNSDR_DESCRIPTOR *yunsdr, RF_CHANNEL ch, int32_t *gain_db);

/* Get the RX RF bandwidth. */
extern int32_t yunsdr_get_rx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *bandwidth_hz);

/* Get current RX sampling frequency. */
extern int32_t yunsdr_get_rx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *sampling_freq_hz);

/* Get current RX LO frequency. */
extern int32_t yunsdr_get_rx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr, uint64_t *lo_freq_hz);

/* Get the gain control mode for the selected channel. */
extern int32_t yunsdr_get_rx_gain_control_mode (YUNSDR_DESCRIPTOR *yunsdr, RF_CHANNEL ch, uint8_t *gc_mode);

/* Get current transmit attenuation for the selected channel. */
extern int32_t yunsdr_get_tx_attenuation (YUNSDR_DESCRIPTOR *yunsdr, RF_CHANNEL ch, uint32_t *attenuation_mdb);

/* Get the TX RF bandwidth. */
extern int32_t yunsdr_get_tx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *bandwidth_hz);

/* Get current TX sampling frequency. */
extern int32_t yunsdr_get_tx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr, uint32_t *sampling_freq_hz);

/* Get current TX LO frequency. */
extern int32_t yunsdr_get_tx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr, uint64_t *lo_freq_hz);

/* Set the receive RF gain for the selected channel. */
extern int32_t yunsdr_set_rx_rf_gain (YUNSDR_DESCRIPTOR *yunsdr,
        RF_CHANNEL ch, int32_t gain_db);

/* Set the RX RF bandwidth. */
extern int32_t yunsdr_set_rx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t bandwidth_hz);

/* Set the RX sampling frequency. */
extern int32_t yunsdr_set_rx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t sampling_freq_hz);

/* Set the RX LO frequency. */
extern int32_t yunsdr_set_rx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint64_t lo_freq_hz);

/* Set the gain control mode for the selected channel. */
extern int32_t yunsdr_set_rx_gain_control_mode (YUNSDR_DESCRIPTOR *yunsdr,
        RF_CHANNEL ch, RF_GAIN_CTRL_MODE gc_mode);

/* Set the transmit attenuation for the selected channel. */
extern int32_t yunsdr_set_tx_attenuation (YUNSDR_DESCRIPTOR *yunsdr,
        RF_CHANNEL ch, uint32_t attenuation_mdb);

/* Set the TX RF bandwidth. */
extern int32_t yunsdr_set_tx_rf_bandwidth (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t  bandwidth_hz);

/* Set the TX sampling frequency. */
extern int32_t yunsdr_set_tx_sampling_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t sampling_freq_hz);

/* Set the TX LO frequency. */
extern int32_t yunsdr_set_tx_lo_freq (YUNSDR_DESCRIPTOR *yunsdr,
        uint64_t lo_freq_hz);

extern int32_t yunsdr_set_ref_clock (YUNSDR_DESCRIPTOR *yunsdr,
        REF_SELECT select);

extern int32_t yunsdr_set_vco_select (YUNSDR_DESCRIPTOR *yunsdr,
        VCO_CAL_SELECT select);

extern int32_t yunsdr_set_trx_select (YUNSDR_DESCRIPTOR *yunsdr,
        TRX_SWITCH select);

extern int32_t yunsdr_set_duplex_select (YUNSDR_DESCRIPTOR *yunsdr,
        DUPLEX_SELECT select);

extern int32_t yunsdr_set_adf4001 (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t val);

extern int32_t yunsdr_set_auxdac (YUNSDR_DESCRIPTOR *yunsdr,
        uint32_t mV);

extern int32_t yunsdr_enable_timestamp (YUNSDR_DESCRIPTOR *yunsdr);

extern int32_t yunsdr_disable_timestamp (YUNSDR_DESCRIPTOR *yunsdr);

extern int32_t yunsdr_read_timestamp (YUNSDR_DESCRIPTOR *yunsdr,
        uint64_t *timestamp);

extern int32_t yunsdr_enable_rx(YUNSDR_DESCRIPTOR *yunsdr, uint32_t nbyte_per_packet,
        uint8_t ch, uint8_t enable);

extern int32_t yunsdr_enable_tx(YUNSDR_DESCRIPTOR *yunsdr, uint32_t nbyte_per_packet,
        uint8_t ch, uint8_t enable);

extern int32_t yunsdr_write_submit(YUNSDR_DESCRIPTOR *yunsdr,
        uint8_t *buffer, uint32_t nbyte, uint8_t ch, uint64_t timestamp);

extern int32_t yunsdr_read_samples(YUNSDR_DESCRIPTOR *yunsdr,
        uint8_t *buf, uint32_t nbyte, uint8_t ch,uint64_t *timestamp);

extern int32_t yunsdr_write_samples(YUNSDR_DESCRIPTOR *yunsdr,
        uint8_t *buf, uint32_t nbyte, uint8_t ch,uint64_t timestamp);

extern void float_to_int16(int16_t *dst, const float *src, int n, float mult);
//extern void int16_to_float(float *dst, const int16_t *src, int len);
extern void int16_to_float(float *dst, const int16_t *src, int len, float mult);

#define __DEBUG__
#ifdef __DEBUG__
#include <stdarg.h>

enum {
    DEBUG_ERR,
    DEBUG_WARN,
    DEBUG_INFO,
};

#define DEBUG_OUTPUT_LEVEL DEBUG_INFO
void debug(int level, const char *fmt, ...);
#endif

#endif /*  __YUNSDR_API_H__ */
