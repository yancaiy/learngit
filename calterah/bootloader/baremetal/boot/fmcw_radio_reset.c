
#include "embARC.h"
#include "system.h"
#include "fmcw_radio_reset.h"

static uint8_t fmcw_radio_switch_bank(void *radio, uint8_t bank);
static char fmcw_radio_reg_read(void *radio, char addr);
void fmcw_radio_reg_write(void *radio, char addr, char data);
static uint32_t radio_spi_cmd_read(char addr);
static void radio_spi_cmd_write(char addr, char data);
static uint32_t radio_spi_cmd_mode(uint32_t mode);

void fmcw_radio_reboot_cause_set(uint32_t cause)
{
        uint8_t buf[4] = {0};

        buf[0] = (cause >> 24) & 0xFF;
        buf[1] = (cause >> 16) & 0xFF;
        buf[2] = (cause >> 8) & 0xFF;
        buf[3] = cause & 0xFF;

        fmcw_radio_switch_bank(NULL, 4);
        fmcw_radio_reg_write(NULL, 0x7B, buf[0]);
        fmcw_radio_reg_write(NULL, 0x7C, buf[1]);
        fmcw_radio_reg_write(NULL, 0x7D, buf[2]);
        fmcw_radio_reg_write(NULL, 0x7E, buf[3]);
        fmcw_radio_switch_bank(NULL, 0);
}

uint32_t fmcw_radio_reboot_cause(void)
{
        uint32_t value;
        uint8_t buf[4] = {0};

        fmcw_radio_switch_bank(NULL, 4);
        buf[0] = fmcw_radio_reg_read(NULL, 0x7B);
        buf[1] = fmcw_radio_reg_read(NULL, 0x7C);
        buf[2] = fmcw_radio_reg_read(NULL, 0x7D);
        buf[3] = fmcw_radio_reg_read(NULL, 0x7E);
        value = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | (buf[3]);
        fmcw_radio_switch_bank(NULL, 0);

        return value;
}

void fmcw_radio_reset(void)
{
       fmcw_radio_switch_bank(NULL, 0);
       fmcw_radio_reg_write(NULL, RADIO_BK0_TEST_CBC2, 0x30 );
       chip_hw_udelay(1000);
       fmcw_radio_reg_write(NULL, RADIO_BK0_TEST_CBC2, 0x00 );
}

static uint8_t fmcw_radio_switch_bank(void *radio, uint8_t bank)
{
        uint8_t old_bank = fmcw_radio_reg_read(NULL, RADIO_BK_REG_BANK);
        fmcw_radio_reg_write(NULL, RADIO_BK_REG_BANK, bank);
        return old_bank;
}

static char fmcw_radio_reg_read(void *radio, char addr)
{
        uint32_t cmd_mode_pre;
        char cmd_rd_data;
        cmd_mode_pre = radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        cmd_rd_data = radio_spi_cmd_read(addr);
        radio_spi_cmd_mode(cmd_mode_pre);
        return cmd_rd_data;
}

void fmcw_radio_reg_write(void *radio, char addr, char data)
{
        uint32_t cmd_mode_pre;
        cmd_mode_pre = radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        radio_spi_cmd_write(addr, data);
        radio_spi_cmd_mode(cmd_mode_pre);
}

static uint32_t radio_spi_cmd_mode(uint32_t mode)
{
        volatile uint32_t *dest = (uint32_t *)RADIO_SPI_CMD_SRC_SEL;
        *dest = mode;
        return mode;
}

static uint32_t radio_spi_cmd_read(char addr)
{
        volatile uint32_t *dest = (uint32_t *)RADIO_SPI_CMD_OUT;
        *dest = (addr & RADIO_SPI_CMD_OUT_ADDR_MASK) << RADIO_SPI_CMD_OUT_ADDR_SHIFT;
        dest = (uint32_t *)RADIO_SPI_CMD_IN;
        return *dest;
}

static void radio_spi_cmd_write(char addr, char data)
{
        volatile uint32_t *dest = (uint32_t *)RADIO_SPI_CMD_OUT;
        *dest  = (  (RADIO_SPI_CMD_OUT_WR_EN_MASK << RADIO_SPI_CMD_OUT_WR_EN_SHIFT)
                  + ((addr & RADIO_SPI_CMD_OUT_ADDR_MASK) << RADIO_SPI_CMD_OUT_ADDR_SHIFT)
                  + ((data & RADIO_SPI_CMD_OUT_DATA_MASK) << RADIO_SPI_CMD_OUT_DATA_SHIFT) );
}

char fmcw_radio_reg_read_field(void *radio, char addr, char shift, char mask)
{
        uint32_t cmd_mode_pre;
        char cmd_rd_data;
        cmd_mode_pre = radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        cmd_rd_data = radio_spi_cmd_read(addr);
        radio_spi_cmd_mode(cmd_mode_pre);
        return ((cmd_rd_data >> shift) & mask);
}