#ifndef FMCW_RADIO_RESET_H
#define FMCW_RADIO_RESET_H

#define RADIO_BK_REG_BANK              0x0
#define RADIO_BK0_TEST_CBC2            0x9

#define RADIO_SPI_CMD_SRC_SEL          0xBA0000
#define RADIO_SPI_CMD_EXT_SPI          0x0
#define RADIO_SPI_CMD_FMCW             0x1
#define RADIO_SPI_CMD_CPU              0x2

#define RADIO_SPI_CMD_OUT              0xBA0004
#define RADIO_SPI_CMD_OUT_WR_EN_SHIFT  15
#define RADIO_SPI_CMD_OUT_WR_EN_MASK   0x1
#define RADIO_SPI_CMD_OUT_ADDR_SHIFT   8
#define RADIO_SPI_CMD_OUT_ADDR_MASK    0x7f
#define RADIO_SPI_CMD_OUT_DATA_SHIFT   0
#define RADIO_SPI_CMD_OUT_DATA_MASK    0xff

#define RADIO_SPI_CMD_IN               0xBA0008
#define RADIO_SPI_CMD_OUT_DATA_SHIFT   0
#define RADIO_SPI_CMD_OUT_DATA_MASK    0xff

void fmcw_radio_reboot_cause_set(uint32_t cause);
uint32_t fmcw_radio_reboot_cause(void);
void fmcw_radio_reset(void);
char fmcw_radio_reg_read_field(void *radio, char addr, char shift, char mask);
void fmcw_radio_reg_write(void *radio, char addr, char data);

#endif
