
#ifndef __K7_PCIE_HW_H__
#define __K7_PCIE_HW_H__
#include <stdint.h>

#define CMD_RF_BASE 0xF0220000

#define CMD_SET_TX_FREQ         (CMD_RF_BASE|3<<8)
#define CMD_SET_TX_SAMPLING     (CMD_RF_BASE|5<<8)
#define CMD_SET_TX_BW           (CMD_RF_BASE|7<<8)
#define CMD_SET_TX_ATTEN1       (CMD_RF_BASE|9<<8)
#define CMD_SET_TX_ATTEN2       (CMD_RF_BASE|11<<8)
#define CMD_SET_RX_FREQ         (CMD_RF_BASE|15<<8)
#define CMD_SET_RX_SAMPLING     (CMD_RF_BASE|17<<8)
#define CMD_SET_RX_BW           (CMD_RF_BASE|19<<8)
#define CMD_SET_RX_GC1          (CMD_RF_BASE|21<<8)
#define CMD_SET_RX_GC2          (CMD_RF_BASE|23<<8)
#define CMD_SET_RX_GAIN1        (CMD_RF_BASE|25<<8)
#define CMD_SET_RX_GAIN2        (CMD_RF_BASE|27<<8)
#define CMD_SET_AUXDAC1         (CMD_RF_BASE|29<<8)
#define CMD_SET_CLK_REF         (CMD_RF_BASE|40<<8)
#define CMD_SET_VCO_REF         (CMD_RF_BASE|41<<8)
#define CMD_SET_DUPLEX          (CMD_RF_BASE|42<<8)
#define CMD_SET_TRX_SWITCH      (CMD_RF_BASE|43<<8)
#define CMD_SET_ADF4001         (CMD_RF_BASE|44<<8)

#define CMD_SET_RX_CHANNEL      0xF0200000
#define CMD_SET_MTIME_START     0xF0210000
#define CMD_SET_SAMPLING_PPS    0xF0230000
#define CMD_SET_RX_BURST_LEN    0xF0240000

#define MASK_CHANNEL  0x3

typedef struct cmd {
    uint32_t value;
    uint32_t type;
}YUNSDR_CMD;

#endif


