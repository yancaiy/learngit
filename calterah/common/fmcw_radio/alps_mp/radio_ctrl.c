
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef UNIT_TEST
#include "calterah_unit_test.h"
#else
#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#endif

#include "calterah_error.h"
#include "sensor_config.h"
#include "baseband.h"
#include "baseband_hw.h"
#include "cmd_reg.h"
#include "radio_reg.h"
#include "radio_ctrl.h"
#include "alps_dmu_reg.h"
#include "baseband_alps_FM_reg.h"
#include "cascade.h"
#include "bb_flow.h"
#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif

#define RADIO_WRITE_BANK_REG_OFFSET(bk, addr, offset, val) fmcw_radio_reg_write(radio, R##bk##_##addr + offset, val)
#define RADIO_READ_BANK_REG_FIELD(bk, addr, field) fmcw_radio_reg_read_field(radio, R##bk##_##addr, \
                                                                             R##bk##_##addr##_##field##_S, \
                                                                             R##bk##_##addr##_##field##_M)

#define RADIO_READ_BANK_CH_REG_FIELD(bk, ch, addr, field) \
        fmcw_radio_reg_read_field(radio,                                \
                                  (ch == 0) ? R##bk##_CH0_##addr : \
                                  (ch == 1) ? R##bk##_CH1_##addr : \
                                  (ch == 2) ? R##bk##_CH2_##addr : \
                                  R##bk##_CH3_##addr,            \
                                  (ch == 0) ? R##bk##_CH0_##addr##_##field##_S : \
                                  (ch == 1) ? R##bk##_CH1_##addr##_##field##_S : \
                                  (ch == 2) ? R##bk##_CH2_##addr##_##field##_S : \
                                  R##bk##_CH3_##addr##_##field##_S, \
                                  (ch == 0) ? R##bk##_CH0_##addr##_##field##_M : \
                                  (ch == 1) ? R##bk##_CH1_##addr##_##field##_M : \
                                  (ch == 2) ? R##bk##_CH2_##addr##_##field##_M : \
                                  R##bk##_CH3_##addr##_##field##_M)


#define RADIO_WRITE_REG(addr, val) fmcw_radio_reg_write(radio, RADIO_##addr, val)

#define RADIO_MOD_REG(addr, field, val) fmcw_radio_reg_mod(radio, RADIO##_##addr, RADIO##_##addr##_##field##_S, \
                                                      addr##_##field##_M, val)
#define RADIO_MOD_BANK_CH_REG(bk, ch, addr, field, val) \
        fmcw_radio_reg_mod(radio, \
                           (ch == 0) ? R##bk##_CH0_##addr :      \
                           (ch == 1) ? R##bk##_CH1_##addr :      \
                           (ch == 2) ? R##bk##_CH2_##addr :      \
                           R##bk##_CH3_##addr,                   \
                           (ch == 0) ? R##bk##_CH0_##addr##_##field##_S : \
                           (ch == 1) ? R##bk##_CH1_##addr##_##field##_S : \
                           (ch == 2) ? R##bk##_CH2_##addr##_##field##_S : \
                           R##bk##_CH3_##addr##_##field##_S, \
                           (ch == 0) ? R##bk##_CH0_##addr##_##field##_M : \
                           (ch == 1) ? R##bk##_CH1_##addr##_##field##_M : \
                           (ch == 2) ? R##bk##_CH2_##addr##_##field##_M : \
                           R##bk##_CH3_##addr##_##field##_M,  \
                           val)

#define TX_EN_CH(ch)                      (ch == 0) ? R1_CH0_TX_EN0 : \
                                          (ch == 1) ? R1_CH1_TX_EN0 : \
                                          (ch == 2) ? R1_CH2_TX_EN0 : \
                                          R1_CH3_TX_EN0

#define TX_PHASE0_CH(ch)                  (ch == 0) ? R1_CH0_TX_TN0 : \
                                          (ch == 1) ? R1_CH1_TX_TN0 : \
                                          (ch == 2) ? R1_CH2_TX_TN0 : \
                                          R1_CH3_TX_TN0
#define TX_PHASE1_CH(ch)                  (ch == 0) ? R1_CH0_TX_TN1 : \
                                          (ch == 1) ? R1_CH1_TX_TN1 : \
                                          (ch == 2) ? R1_CH2_TX_TN1 : \
                                          R1_CH3_TX_TN1
/* 01       10                01         10 */
/* in phase opposite phase    in phase opposite phase */
#define VAM_TX_BPM_PATTEN_1               0x66
/* 01       01                10         10 */
/* in phase, in phase, opposite phase, opposite phase */
#define VAM_TX_BPM_PATTEN_2               0x5A
/* 01       10                10         01*/
/* in phase, opposite phase, opposite phase, in phase */
#define VAM_TX_BPM_PATTEN_3               0x69

#define RADIO_WRITE_BANK_FWCW_TX_REG(bk, tx, addr, val) \
        fmcw_radio_reg_write(radio, \
                           (tx == 0) ? R##bk##_FMCW_TX0_##addr :      \
                           (tx == 1) ? R##bk##_FMCW_TX1_##addr :      \
                           (tx == 2) ? R##bk##_FMCW_TX2_##addr :      \
                           R##bk##_FMCW_TX3_##addr,                   \
                           val)

#ifdef UNIT_TEST
#define MDELAY(ms)
#else
#define MDELAY(ms)  chip_hw_mdelay(ms);
#endif

static uint8_t vam_status[4];
static uint8_t txphase_status[8];
#ifdef FUNC_SAFETY
bool safety_monitor_mode = false;
bool rf_loopback_mode = false;
#endif
#if REFPLL_CBANK == 1
volatile float auto_lock_junc_temp = 0.0f;
#endif
#define REF_PLL_RELOCK_CNT (2)

#define TS_K 107.04
#define TS_D 107170.0
#define TS_T 125

static float ts_coefficient[3] = {TS_K, TS_D, TS_T};

uint32_t fmcw_radio_compute_lock_freq(fmcw_radio_t *radio);

uint32_t radio_spi_cmd_mode(uint32_t mode)
{
        volatile uint32_t *dest = (uint32_t *)RADIO_SPI_CMD_SRC_SEL;
        *dest = mode;
        return mode;
}

void radio_spi_cmd_write(char addr, char data)
{
        volatile uint32_t *dest = (uint32_t *)RADIO_SPI_CMD_OUT;
        *dest  = (  (RADIO_SPI_CMD_OUT_WR_EN_MASK << RADIO_SPI_CMD_OUT_WR_EN_SHIFT)
                  + ((addr & RADIO_SPI_CMD_OUT_ADDR_MASK) << RADIO_SPI_CMD_OUT_ADDR_SHIFT)
                  + ((data & RADIO_SPI_CMD_OUT_DATA_MASK) << RADIO_SPI_CMD_OUT_DATA_SHIFT) );
}

uint32_t radio_spi_cmd_read(char addr)
{
        volatile uint32_t *dest = (uint32_t *)RADIO_SPI_CMD_OUT;
        *dest = (addr & RADIO_SPI_CMD_OUT_ADDR_MASK) << RADIO_SPI_CMD_OUT_ADDR_SHIFT;
        dest = (uint32_t *)RADIO_SPI_CMD_IN;
        return *dest;
}

EMBARC_NOINLINE char fmcw_radio_reg_read(fmcw_radio_t *radio, char addr)
{
        uint32_t cmd_mode_pre;
        char cmd_rd_data;
        cmd_mode_pre = radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        cmd_rd_data = radio_spi_cmd_read(addr);
        radio_spi_cmd_mode(cmd_mode_pre);
        return cmd_rd_data;
}

EMBARC_NOINLINE char fmcw_radio_reg_read_field(fmcw_radio_t *radio, char addr, char shift, char mask)
{
        uint32_t cmd_mode_pre;
        char cmd_rd_data;
        cmd_mode_pre = radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        cmd_rd_data = radio_spi_cmd_read(addr);
        radio_spi_cmd_mode(cmd_mode_pre);
        return ((cmd_rd_data >> shift) & mask);
}

EMBARC_NOINLINE void fmcw_radio_reg_write(fmcw_radio_t *radio, char addr, char data)
{
        uint32_t cmd_mode_pre;
        cmd_mode_pre = radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        radio_spi_cmd_write(addr, data);
        radio_spi_cmd_mode(cmd_mode_pre);
}

EMBARC_NOINLINE void fmcw_radio_reg_mod(fmcw_radio_t *radio, char addr, char shift, char mask, char data)
{
        uint32_t cmd_mode_pre;
        char cmd_rd_data;
        cmd_mode_pre = radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        cmd_rd_data = radio_spi_cmd_read(addr);
        cmd_rd_data &= ~(mask << shift);
        cmd_rd_data |= (data & mask) << shift;
        radio_spi_cmd_write(addr, cmd_rd_data);
        radio_spi_cmd_mode(cmd_mode_pre);
}

void fmcw_radio_reg_dump(fmcw_radio_t *radio)
{
        uint8_t old_bank;
        char bank_index, addr_index, rd_data;
        old_bank = fmcw_radio_switch_bank(radio, 0);
        for (bank_index = 0; bank_index < BNK_NUM; bank_index++){
                fmcw_radio_switch_bank(radio, bank_index);
                for (addr_index = 1; addr_index < BNK_SIZE; addr_index++) {
                        rd_data = fmcw_radio_reg_read(radio, addr_index);
                        EMBARC_PRINTF("\r\n%1d %3d 0x%2x",
                                      bank_index, addr_index, rd_data);
                }
                                MDELAY(10);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

uint8_t fmcw_radio_switch_bank(fmcw_radio_t *radio, uint8_t bank)
{
        uint8_t old_bank = RADIO_READ_BANK_REG(0, REG_BANK);
        RADIO_WRITE_BANK_REG(0, REG_BANK, bank);
        return old_bank;
}

int32_t fmcw_radio_rx_buffer_on(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);
        int ch;
        RADIO_MOD_BANK_REG(2, LVDS_LDO25, LDO25_LVDS_LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(2, LVDS_LDO25, LDO25_LVDS_VSEL, 0x4);
        for (ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_CH_REG(2, ch, BUFFER, EN, 0x1);
        }
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_ldo_on(fmcw_radio_t *radio)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        /* CBC enable */
        RADIO_MOD_BANK_REG(0, CBC_EN, CGM_EN, 0x1);
        RADIO_MOD_BANK_REG(0, CBC_EN, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, CBC_EN, BG_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO25_PMU, EN, 0x1);
        /* Output voltage fixed in AlpsMP */
#if HTOL_T == 1
        RADIO_MOD_BANK_REG(0, POR,LDO11_SPI_VO_SEL, 0x6);
#else
        RADIO_MOD_BANK_REG(0, POR,LDO11_SPI_VO_SEL, 0x4);
#endif
        /* LDO for PLL enable */
        RADIO_MOD_BANK_REG(0, RP_LDO0, LDO25_XTAL_LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_LDO3, LDO11_MD_LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_LDO4, LDO11_MD2_LDO_EN, 0x1);

#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER) {
                RADIO_MOD_BANK_REG(0, FP_LDO1, LDO25_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_LDO2, LDO11_VC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_LDO3, LDO11_MD_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_LDO4, LDO11_CM_EN, 0x1);
        } else {
                RADIO_MOD_BANK_REG(0, FP_LDO1, LDO25_EN, 0x0);
                RADIO_MOD_BANK_REG(0, FP_LDO2, LDO11_VC_EN, 0x0);
                RADIO_MOD_BANK_REG(0, FP_LDO3, LDO11_MD_EN, 0x0);
        }
        RADIO_WRITE_REG(BK0_LO_LDO1, 0xc0);
#else
        RADIO_MOD_BANK_REG(0, FP_LDO1, LDO25_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LDO2, LDO11_VC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LDO3, LDO11_MD_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LDO4, LDO11_CM_EN, 0x1);
#endif
        /* LDO for LO enable */
        RADIO_MOD_BANK_REG(0, LO_LDO0, LDO11_LMP_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LO_LDO2, LDO11_TXLO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LO_LDO3, LDO11_RXLO_EN, 0x1);
        /* LDO for RX enable */
        RADIO_MOD_BANK_REG(0, RX_LDO0, LDO11_RFN_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RX_LDO1, LDO11_RFS_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RX_LDO2, LDO25_BBN_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RX_LDO3, LDO25_BBS_EN, 0x1);

        fmcw_radio_switch_bank(radio, 1);
        /* LDO for TX and TX's PA enable */
        if (cfg->tx_groups[0] != 0)        {
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX0_EN, 0x1);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX0_PA_EN, 0x1);
        }

        if (cfg->tx_groups[1] != 0){
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX1_EN, 0x1);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX1_PA_EN, 0x1);

        }

        if (cfg->tx_groups[2] != 0){
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX2_EN, 0x1);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX2_PA_EN, 0x1);
        }

        if (cfg->tx_groups[3] != 0){
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX3_EN, 0x1);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX3_PA_EN, 0x1);
        }

        /* Disable LDO for TX and TX's PA in cascade slave chip */
#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_SLAVE) {
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX0_EN, 0x0);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX0_PA_EN, 0x0);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX1_EN, 0x0);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX1_PA_EN, 0x0);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX2_EN, 0x0);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX2_PA_EN, 0x0);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX3_EN, 0x0);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX3_PA_EN, 0x0);
        }
#endif
        /* No Need to ON Buffer, only in test */
        /* LDO for ADC enable */
        RADIO_MOD_BANK_REG(1, ADC_LDO0, LDO11_ADC12_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO1, LDO12_ADC12_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO2, LDO25_ADC12_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO3, LDO11_ADC34_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO4, LDO12_ADC34_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO5, LDO25_ADC34_EN, 0x1);

        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_refpll_on(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        /* enable reference PLL's mainly part */
#ifdef CHIP_CASCADE
        /* cfg->cascade_mode must not be used here, since initializaiton is not started yet */
        if (chip_cascade_status() == CHIP_CASCADE_MASTER)
                RADIO_MOD_BANK_REG(0, RP_EN, CLK_400M_FMCW_EN, 0x1);
        else
                RADIO_MOD_BANK_REG(0, RP_EN, CLK_400M_FMCW_EN, 0x0);

#else
        RADIO_MOD_BANK_REG(0, RP_EN, CLK_400M_FMCW_EN, 0x1);
#endif
        RADIO_MOD_BANK_REG(0, RP_EN, DIV_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_EN, VC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_EN, CP_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_EN, XOSC_BUF_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RP_EN, XOSC_EN, 0x1); // FIXME, slave may change to be 0, but 1 may also work, comments by wenting
        //enalbe clock to Lock detector,400M ADC, CPU
        RADIO_MOD_BANK_REG(0, PLL_VIO_O, RP_REF_AUX_O_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_VIO_O, RP_REF_O_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_400M_ADC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_SDMADC_CLKSEL, 0x0);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_LK_DIV_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_LK_REF_EN, 0x1);
        //Since PN under 450MHz of ALpsB is very poor, abandon 450MHz, default 400MHz
        #if FMCW_SDM_FREQ == 400
                RADIO_MOD_BANK_REG(0, RP_DIV, L, 0x1);
                RADIO_MOD_BANK_REG(0, RP_LF, DIVH2, 0x2);
        #elif FMCW_SDM_FREQ == 360
                RADIO_MOD_BANK_REG(0, RP_DIV, L, 0x0);
                RADIO_MOD_BANK_REG(0, RP_LF, DIVH2, 0x4);
        #endif

        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_pll_on(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        //enable FP's mainly block and on 4G
#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER) {
                RADIO_MOD_BANK_REG(0, FP_EN, TSPCDIV_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_EN, CMLDIV_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_EN, 4G_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_EN, VC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_EN, CP_EN, 0x1);
                RADIO_MOD_BANK_REG(0, FP_EN, PFD_DL_DIS, 0x0); // auto dis should open, or FMCW will not work, comments by wenting
                RADIO_MOD_BANK_REG(0, FP_VC, FP_PFD_DL_DIS2, 0x1); //dis2
        } else {
                RADIO_WRITE_REG(BK0_FP_EN, 0);
        }
#else
        RADIO_MOD_BANK_REG(0, FP_EN, TSPCDIV_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_EN, CMLDIV_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_EN, 4G_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_EN, VC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_EN, CP_EN, 0x1);
        //dis1's setting at up ramp
        RADIO_MOD_BANK_REG(0, FP_EN, PFD_DL_DIS, 0x0);
        //dis2 = 1
        RADIO_MOD_BANK_REG(0, FP_VC, FP_PFD_DL_DIS2, 0x1); //dis2
#endif
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, FP_LKDIV_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, FP_LKREF_EN, 0x1);
        /* change TSPC value for pll lock yield imporving @ ss corner */
        RADIO_MOD_BANK_REG(0, FP_CML, FP_CML_IB_SEL, 0x8);
        RADIO_MOD_BANK_REG(0, FP_TSPC, FP_FMCWDIV_P, 0xf);
        RADIO_MOD_BANK_REG(0, FP_TSPC, FP_AMP_IB_SEL, 0x8);
        //auto dis1
        fmcw_radio_switch_bank(radio, 3);
        RADIO_MOD_BANK_REG(3, AT_DL_DIS, SEL, 0x0);
        RADIO_MOD_BANK_REG(3, AT_DL_DIS, EN , 0x1);
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_lo_on(fmcw_radio_t *radio, bool enable)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* enable LO's mainly block,including RX and TX */
        RADIO_MOD_BANK_REG(0, LO_EN0, LO_RXDR_STG3_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN0, LO_RXDR_STG2_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN0, LO_RXDR_STG1_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN0, DBL2_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN0, DBL1_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN0, LODR_EN, enable);

#ifdef CHIP_CASCADE
        RADIO_MOD_BANK_REG(0, LO_EN0, VCBUFF2_EN, 0x0);
        RADIO_MOD_BANK_REG(0, LO_EN0, VCBUFF1_EN, 0x0);
        if (enable == true) {
                if (chip_cascade_status() == CHIP_CASCADE_MASTER)
                        RADIO_WRITE_REG(BK0_LO_EN2, 0xff);
                else
                        RADIO_WRITE_REG(BK0_LO_EN2, 0x7f);

        } else {
                        RADIO_WRITE_REG(BK0_LO_EN2, 0x0);
        }
#else
        RADIO_MOD_BANK_REG(0, LO_EN0, VCBUFF2_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN0, VCBUFF1_EN, enable);
#endif

        RADIO_MOD_BANK_REG(0, LO_EN1, LO_VTN_VAR_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG3_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG2_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG1_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG3_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG2_EN, enable);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG1_EN, enable);

        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_tx_on(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        int ch;
        //4 channels
        for (ch = 0; ch < MAX_NUM_TX; ch++) {
                RADIO_MOD_BANK_CH_REG(1, ch, TX_EN0, PADR_BI_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, TX_EN0, PA_BI_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, TX_EN0, QDAC_BI_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, TX_EN0, IDAC_BI_EN, 0x1);
        }
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_rx_on(fmcw_radio_t *radio, bool enable)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;

        if (enable == true){
                fmcw_radio_hp_auto_ch_off(radio,-1);
        }
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t ch;
        uint8_t enable_2bit = 0x3 & (enable | (enable << 1));
        //4 channels
        for (ch = 0; ch < MAX_NUM_RX; ch++) {
#if defined(FUNC_SAFETY) && (SAFETY_FEATURE_SM12 == 1)
                /* if rf_loopback_mode equals to true, rf loopback function will disable rxlobuf and lna2 bias */
                if (rf_loopback_mode == false) {
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, RLB_BI_EN, enable);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, LNA2_BI_EN, enable);
                }
#else
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, RLB_BI_EN, enable);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, LNA2_BI_EN, enable);
#endif
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, LNA1_BI_EN, enable);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, TIA_BI_EN, enable);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, TIA_S1_EN, enable_2bit);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, TIA_VCTRL_EN, enable);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_BB_EN, BI_EN, enable);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_BB_EN, VGA1_EN, enable);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_BB_EN, VGA2_EN, enable);

                /* RX gain setting to cfg value */
                if (enable == true) {
#if defined(FUNC_SAFETY) && ((SAFETY_FEATURE_SM11 == 1) || (SAFETY_FEATURE_SM12 == 1))
                        /* if safety_monitor_mode equals to true, if&rf loopback function will set rx gain to other steps */
                        if (safety_monitor_mode == false) {
                                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, cfg->rf_tia_gain);
                                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, cfg->rf_vga2_gain);
                                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, cfg->rf_vga1_gain);
                        }
#else
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, cfg->rf_tia_gain);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, cfg->rf_vga2_gain);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, cfg->rf_vga1_gain);
#endif
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN2, HP2_SEL, cfg->rf_hpf2);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN2, HP1_SEL, cfg->rf_hpf1);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_BI4, VGA2_VCMSEL, 0x4);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_BI4, VGA1_VCMSEL, 0x4);
                }
        }
#ifndef CHIP_CASCADE  // Not ready for cascade but HPF can still work
        fmcw_radio_hp_auto_ch_on(radio,-1); // To shorten HPF set up time
#endif
        fmcw_radio_switch_bank(radio, old_bank);
#ifndef CHIP_CASCADE
        fmcw_radio_hp_auto_ch_on(radio,-1); /* re-open hp after inter_frame_power_save */
#endif
        return E_OK;
}

int32_t fmcw_radio_adc_on(fmcw_radio_t *radio)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, ADC_MUX_O_SEL, CLK_EN, 0x1);
        int ch;
        //4 channels ADC, vcmbuf should always kept low
        for (ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, RST, 0x0);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, BUFFER_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, BUFFER_VCMBUF_EN, 0x0);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, ANALS_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, OP1_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, OP2_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, OP3_EN, 0x1);

                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, CMP_VCALREF_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, BI_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, IDAC1_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, IDAC3_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, ESL_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, REFPBUF_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, REFNBUF_EN, 0x1);
        }
        fmcw_radio_switch_bank(radio, 2);

        /*400MHz and 800MHz ADC switch*/
        //400MHz  Bank0 Reg0x26 set as 0x4F,
        //Bank1 Reg 0x34, 0x43, 0x52 and 0x61 set as 0x7F;

        //800MHz  Bank0 Reg0x26 set as 0x3F,
        //Bank1 Reg0x34, 0x43, 0x52 and 0x61 set as 0xFF ;
        //Test Plan 901

        //800MHz support from AlpsB
        if (cfg->adc_freq == 20){      //400MHz, signal bandwidth 20MHz
                //400MHz
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_400M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_SDMADC_CLKSEL, 0x0);

                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, CH0_ADC_EN0, BW20M_EN, 0x0);
                RADIO_MOD_BANK_REG(1, CH1_ADC_EN0, BW20M_EN, 0x0);
                RADIO_MOD_BANK_REG(1, CH2_ADC_EN0, BW20M_EN, 0x0);
                RADIO_MOD_BANK_REG(1, CH3_ADC_EN0, BW20M_EN, 0x0);

                fmcw_radio_switch_bank(radio, 2);
                RADIO_MOD_BANK_REG(2, CH1_ADC_FILTER, BDW_SEL, 0x0);
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, CLK_SEL, 0x0);
                //default 0x4
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, DAT_SFT_SEL, 0x4);
        }
        else if (cfg->adc_freq == 25){ //400MHz, signal bandwidth 25MHz
                //400MHz
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_400M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_SDMADC_CLKSEL, 0x0);

                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, CH0_ADC_EN0, BW20M_EN, 0x0);
                RADIO_MOD_BANK_REG(1, CH1_ADC_EN0, BW20M_EN, 0x0);
                RADIO_MOD_BANK_REG(1, CH2_ADC_EN0, BW20M_EN, 0x0);
                RADIO_MOD_BANK_REG(1, CH3_ADC_EN0, BW20M_EN, 0x0);

                fmcw_radio_switch_bank(radio, 2);
                RADIO_MOD_BANK_REG(2, CH1_ADC_FILTER, BDW_SEL, 0x1);
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, CLK_SEL, 0x0);
                //default 0x3 for 16bits output
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, DAT_SFT_SEL, 0x3);
        }
        else if (cfg->adc_freq == 40){ //800MHz, signal bandwidth 40MHz

                //800Mhz
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_400M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_SDMADC_CLKSEL, 0x1);

                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, CH0_ADC_EN0, BW20M_EN, 0x1);
                RADIO_MOD_BANK_REG(1, CH1_ADC_EN0, BW20M_EN, 0x1);
                RADIO_MOD_BANK_REG(1, CH2_ADC_EN0, BW20M_EN, 0x1);
                RADIO_MOD_BANK_REG(1, CH3_ADC_EN0, BW20M_EN, 0x1);

                fmcw_radio_switch_bank(radio, 2);
                RADIO_MOD_BANK_REG(2, CH1_ADC_FILTER, BDW_SEL, 0x0);
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, CLK_SEL, 0x1);
                //default 0x4
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, DAT_SFT_SEL, 0x4);
        }
        else if (cfg->adc_freq == 50){ //800MHz, signal bandwidth 50MHz

                //800Mhz
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_400M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_SDMADC_CLKSEL, 0x1);

                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, CH0_ADC_EN0, BW20M_EN, 0x1);
                RADIO_MOD_BANK_REG(1, CH1_ADC_EN0, BW20M_EN, 0x1);
                RADIO_MOD_BANK_REG(1, CH2_ADC_EN0, BW20M_EN, 0x1);
                RADIO_MOD_BANK_REG(1, CH3_ADC_EN0, BW20M_EN, 0x1);

                fmcw_radio_switch_bank(radio, 2);
                RADIO_MOD_BANK_REG(2, CH1_ADC_FILTER, BDW_SEL, 0x1);
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, CLK_SEL, 0x1);
                //default 0x4
                RADIO_MOD_BANK_REG(2, ADC_FILTER0, DAT_SFT_SEL, 0x3);
        }
        else
        {

        }

        RADIO_MOD_BANK_REG(2, ADC_FILTER0, CMOS_O_EN, 0x1);
        RADIO_MOD_BANK_REG(2, ADC_FILTER0, RSTN, 0x1);
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_do_refpll_cal(fmcw_radio_t *radio)
{
        uint8_t old_bank;
        bool lock_status = 0;
        int8_t relock_cnt = REF_PLL_RELOCK_CNT;

        old_bank = fmcw_radio_switch_bank(radio, 0);
        while (!lock_status && relock_cnt--) {
                //for refernce PLL, release reset is enough
                //Settling time is already been set by default
                RADIO_MOD_BANK_REG(0, AT_LK0, RP_RSTN, 0x0);
                MDELAY(2);
                RADIO_MOD_BANK_REG(0, AT_LK0, RP_RSTN, 0x1);
                MDELAY(15);
                lock_status = fmcw_radio_is_refpll_locked(radio);
        }
        fmcw_radio_switch_bank(radio, old_bank);
        if (lock_status) {
                return E_OK;
        } else {
                return E_REFPLL_UNLOCK;
        }
}

void fmcw_radio_fmcw_pll_hw_lock(fmcw_radio_t *radio)
{
        RADIO_MOD_BANK_REG(0, AT_LK0, FP_RSTN, 0x0);
        RADIO_MOD_BANK_REG(0, AT_LK0, FP_RSTN, 0x1);
        MDELAY(1);
}

int32_t fmcw_radio_fmcw_pll_sw_lock(fmcw_radio_t *radio)
{
        int32_t read_cnt = 0;
        int32_t det_out_min = 0xffff;
        int32_t det_out = 0;
        int32_t cbank_min = 0xf;
        bool lock_valid = false;
        int32_t result = E_OK;

        RADIO_MOD_BANK_REG(0, AT_LK0, FP_BYP, 0x1);
        RADIO_MOD_BANK_REG(0, LK_DTC2_0, LD_RSTN_400M, 0x0);
        MDELAY(1);
        RADIO_MOD_BANK_REG(0, LK_DTC2_0, LD_RSTN_400M, 0x1);

        // open pll loop
        RADIO_MOD_BANK_REG(0, FP_EN, CP_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_SET_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_SETH_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_SETL_EN, 0x1);

        // bank search
        for (int cbank = 0; cbank <= 11; cbank++) {
                lock_valid = false;
                read_cnt = 20;
                RADIO_MOD_BANK_REG(0, FP_VC, FP_CBK, cbank);
                while (!lock_valid && (read_cnt--)) {
                        UDELAY(300);
                        lock_valid = RADIO_READ_BANK_REG(0, LK_DTC2_1);
                        if(lock_valid) {
                                det_out = RADIO_READ_BANK_REG(0, LK_DTC2_2) + \
                                        ( RADIO_READ_BANK_REG(0, LK_DTC2_3) << 8 );
                                if (det_out >= (1 << 15)) {
                                        det_out = abs(det_out - (1 << 16));
                                }
                                if(det_out_min > det_out) {
                                        det_out_min = det_out;
                                        cbank_min = cbank;
                                }
                        }
                }
                if(!lock_valid){
                        result = E_TMOUT;
                        break;
                }
        }

        if(lock_valid) {
                result = E_OK;
                RADIO_MOD_BANK_REG(0, FP_VC, FP_CBK, cbank_min);
        }

        // close pll loop
        RADIO_MOD_BANK_REG(0, FP_EN, CP_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_SET_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_SETH_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_SETL_EN, 0x0);

        return result;
}

int32_t fmcw_radio_do_pll_cal(fmcw_radio_t *radio, uint32_t lock_freq)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5);// PLL bank is not considered
        int32_t result = E_OK;
        bool fmcw_locked = false;
        int8_t lock_cnt_hw = 10;
        int8_t lock_cnt_sw = 10;

        //config frequency
        RADIO_WRITE_BANK_REG(5,FMCW_START_FREQ_1_0, REG_L(lock_freq));
        RADIO_WRITE_BANK_REG(5,FMCW_START_FREQ_1_1, REG_M(lock_freq));
        RADIO_WRITE_BANK_REG(5,FMCW_START_FREQ_1_2, REG_H(lock_freq));
        RADIO_WRITE_BANK_REG(5,FMCW_START_FREQ_1_3, REG_INT(lock_freq));
        //reset to 0 to clean unexpected config
        fmcw_radio_switch_bank(radio, 3);
        RADIO_WRITE_BANK_REG(3, FMCW_SYNC, 0x0);
         RADIO_WRITE_BANK_REG(3, FMCW_START, 0x0);
        //mode to 3'b000 is hold mode, single point frequency
        RADIO_WRITE_BANK_REG(3, FMCW_MODE_SEL, 0x0);

        /* During calibration sync signal is disabled */
        RADIO_MOD_BANK_REG(3, FMCW_SYNC, EN, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, ADDER_RSTN, 0x1);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x1);
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x1);
        //config FP's settling time to the largest
        fmcw_radio_switch_bank(radio, 2);
        RADIO_WRITE_BANK_REG(2, FP_SLTSIZE0, 0xFF);
        RADIO_WRITE_BANK_REG(2, FP_SLTSIZE1, 0xFF);
        //On 3MHz bandwidth for fast settling
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, FP_LDO1, LDO25_VO_SEL, 0x7);
        RADIO_MOD_BANK_REG(0, FP_LF, 3MBW_EN, 0x1);

        // hardware lock
        while(!fmcw_locked && (lock_cnt_hw--)){
                fmcw_radio_fmcw_pll_hw_lock(radio);
                if (fmcw_radio_is_pll_locked(radio)) {
                        fmcw_locked = true;
                }
        }
        // software lock
        while(!fmcw_locked && (lock_cnt_sw--)){
                result = fmcw_radio_fmcw_pll_sw_lock(radio);
                if (!result && fmcw_radio_is_pll_locked(radio)){
                        fmcw_locked = true;
                }
        }
        if(!fmcw_locked){
                return E_FMCW_PLL_UNLOCK;
        }

        fmcw_radio_switch_bank(radio, 3);
        /* Turn off START */
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x1);
        //shut down 3MHz bandwidth
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, FP_LF, 3MBW_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_LDO1, LDO25_VO_SEL, 0x4);
        //added auto 3MHz bandwidth
        fmcw_radio_switch_bank(radio, 3);
        //set auto 3MHz bandwidth to enable at down and idle state
        RADIO_MOD_BANK_REG(3, AT_3MBW, SEL, 0x1);
        //enable auto 3MHz bandwidth
        RADIO_MOD_BANK_REG(3, AT_3MBW, EN , 0x0);
        fmcw_radio_switch_bank(radio, old_bank);
        return result;
}

bool fmcw_radio_is_refpll_locked(fmcw_radio_t *radio)
{
        uint8_t old_bank;
        old_bank = fmcw_radio_switch_bank(radio, 0);
        bool status = RADIO_READ_BANK_REG_FIELD(0,AT_LK1,RP_LCKED);
        fmcw_radio_switch_bank(radio, old_bank);
        return status;
}

bool fmcw_radio_is_pll_locked(fmcw_radio_t *radio)
{
        uint8_t old_bank;
        old_bank = fmcw_radio_switch_bank(radio, 0);
#ifdef CHIP_ALPS_A
        bool status = RADIO_READ_BANK_REG_FIELD(0,AT_LK1,RP_LCKED) &
                      RADIO_READ_BANK_REG_FIELD(0,AT_LK1,FP_LCKED);
#elif CHIP_ALPS_MP
        //fixed analog's routing bug, analog's layout have effected auto lock's logic, confirmed with wenting
        //Not a very solid solution, expect to be solved by analog in the next T.O.
        bool status = RADIO_READ_BANK_REG_FIELD(0,AT_LK1,FP_LCKED) ? RADIO_READ_BANK_REG_FIELD(0,AT_LK1,RP_LCKED) :
                    ( RADIO_READ_BANK_REG(0, LK_DTC2_1) == 1 &&
                      RADIO_READ_BANK_REG(0, LK_DTC2_2) == 0 &&
                      RADIO_READ_BANK_REG(0, LK_DTC2_3) == 0  );
#endif
        fmcw_radio_switch_bank(radio, old_bank);
        return status;
}

/* enable clock to CPU */
int32_t fmcw_radio_pll_clock_en(void)
{
        fmcw_radio_t *radio = NULL;
        int32_t result = E_OK;
        uint8_t old_bank;
        old_bank = fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, CBC_EN, CGM_EN, 0x01);
        RADIO_MOD_BANK_REG(0, CBC_EN, LDO_EN, 0x01);
        RADIO_MOD_BANK_REG(0, CBC_EN, BG_EN, 0x01);

        RADIO_MOD_BANK_REG(0, LDO25_PMU, EN, 0x01);

        RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_LDO_EN, 0x01);
        RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_LDO_EN, 0x01);
        RADIO_MOD_BANK_REG(0, RP_LDO3, LDO11_MD_LDO_EN, 0x01);
        RADIO_MOD_BANK_REG(0, RP_LDO4, LDO11_MD2_LDO_EN, 0x01);

        RADIO_MOD_BANK_REG(0, RP_LF, DIVCPU_EN, 0x01);
        RADIO_MOD_BANK_REG(0, RP_LF, DIVDIG_EN, 0x01);

        fmcw_radio_refpll_on(radio);
        MDELAY(1);

        result = fmcw_radio_do_refpll_cal(radio);
        fmcw_radio_switch_bank(radio, old_bank);
        return result;
}

void fmcw_radio_frame_interleave_reg_write(fmcw_radio_t *radio, uint32_t fil_que, uint8_t fil_prd)
{
        /* config the frame loop registers */
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        RADIO_WRITE_BANK_REG(3, FMCW_FIL0, REG_L(fil_que));
        RADIO_WRITE_BANK_REG(3, FMCW_FIL1, REG_M(fil_que));
        RADIO_WRITE_BANK_REG(3, FMCW_FIL2, REG_H(fil_que));
        RADIO_WRITE_BANK_REG(3, FMCW_FIL3, REG_INT(fil_que));
        RADIO_WRITE_BANK_REG(3, FMCW_FIL_PRD,  fil_prd);
        if (NUM_FRAME_TYPE == 1){
                RADIO_WRITE_BANK_REG(3, FMCW_FIL_EN, 0x0);
        }
        else{
                RADIO_WRITE_BANK_REG(3, FMCW_FIL_EN, 0x1);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_frame_interleave_pattern(fmcw_radio_t *radio, uint8_t frame_loop_pattern)
{
        /* set frame interleave pattern */
        uint32_t  fil_que   = frame_loop_pattern;
        uint32_t  fil_prd   = 0;

        fmcw_radio_frame_interleave_reg_write(radio, fil_que, fil_prd);
}

void fmcw_radio_frame_type_reset(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        RADIO_WRITE_BANK_REG(3, FMCW_FIL_EN, 0x0); // reset asserted
        RADIO_WRITE_BANK_REG(3, FMCW_FIL_EN, 0x1); // reset deasserted
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_generate_fmcw(fmcw_radio_t *radio)
{
        int32_t result = E_OK;
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
#ifndef CHIP_CASCADE
        fmcw_radio_hp_auto_ch_on(radio,-1);
#endif
        /* stop fmcw before programming */
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x0);
        /* tx phase */
        if ((radio->frame_type_id) == 0) {
                uint32_t lock_freq = fmcw_radio_compute_lock_freq(radio);
                result = fmcw_radio_do_pll_cal(radio, lock_freq); /* only run in frame_type 0, do not run in other cases*/
                if (E_OK != result){
                        return result;
                }
        }

        /*Configure all Txs Phase Shifter */
        uint32_t ch, reg_val;
        for (ch = 0; ch < MAX_NUM_TX; ch++) /* config one time*/
        {
            reg_val = phase_val_2_reg_val(cfg->tx_phase_value[ch]);
            fmcw_radio_set_tx_phase(radio, ch, reg_val);
        }

        /* chirp parameters */
        fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_0,   REG_L(radio->start_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_1,   REG_M(radio->start_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_2,   REG_H(radio->start_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_3,   REG_INT(radio->start_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_0,    REG_L(radio->stop_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_1,    REG_M(radio->stop_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_2,    REG_H(radio->stop_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_3,    REG_INT(radio->stop_freq));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_0, REG_L(radio->step_up));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_1, REG_M(radio->step_up));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_2, REG_H(radio->step_up));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_3, REG_INT(radio->step_up));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_0, REG_L(radio->step_down));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_1, REG_M(radio->step_down));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_2, REG_H(radio->step_down));
        fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_3, REG_INT(radio->step_down));
        fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_0,         REG_L(radio->cnt_wait));
        fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_1,         REG_M(radio->cnt_wait));
        fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_2,         REG_H(radio->cnt_wait));
        fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_3,         REG_INT(radio->cnt_wait));
        fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_0,   REG_L(radio->nchirp));
        fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_1,   REG_M(radio->nchirp));
        fmcw_radio_switch_bank(radio, 3);

        /* MIMO */

        /*turn off VAM mode,ps mode,agc mode bofore turning on to protection switch modes without power off the chip*/
        fmcw_radio_special_mods_off(radio); /* only clear one time */

        /* virtual array settings */
        if (cfg->nvarray >= 2) {
                fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
                for (int ch = 0; ch < MAX_NUM_TX; ch++) {
                        if (cfg->tx_groups[ch]>0) {
                                // set VAM config
                                uint8_t vam_cfg = 0;
                                /*Bug fix for anti velamb glitch*/
                                if (cfg->anti_velamb_en)
                                {
                                        if((cfg->nvarray >= 3) && cfg->agc_mode)
                                        {
                                                vam_cfg = ( 0x61 | (cfg->anti_velamb_en << 3) | (( cfg->nvarray - 1 ) << 1 ));
                                        }
                                        else
                                        {
                                                vam_cfg = ( 0x41 | (cfg->anti_velamb_en << 3) | (( cfg->nvarray - 1 ) << 1 ));
                                        }
                                }
                                else
                                {
                                        vam_cfg = ( 0x61 | (cfg->anti_velamb_en << 3) | (( cfg->nvarray - 1 ) << 1 ));
                                }
                                fmcw_radio_reg_write(radio, R5_FMCW_TX0_CTRL_1_2 + ch * 3, vam_cfg);

                                /* set VAM group patten*/
                                uint16_t bit_mux[MAX_NUM_TX] = {0,0,0,0};
                                bit_parse(cfg->tx_groups[ch], bit_mux);
                                uint8_t bit_mux_all = 0;
                                /*Bug fix for anti velamb glitch*/
                                if ((cfg->anti_velamb_en == false) || ((cfg->nvarray >= 3) && cfg->agc_mode)){
                                        bit_mux_all = bit_mux[0]<<6 | bit_mux[1]<<4 | bit_mux[2]<< 2 | bit_mux[3];
                                }
                                else if( cfg->phase_scramble_on ){
                                        bit_mux_all = 0xff ;
                                } else{
                                        /*Do Nothing*/
                                }
                                fmcw_radio_reg_write(radio, R5_FMCW_TX0_CTRL_1_1 + ch * 3, bit_mux_all);
                        } else {
                                fmcw_radio_reg_write(radio, R5_FMCW_TX0_CTRL_1_2 + ch * 3, 0); // clear 0
                        }
                }
        }

        /* phase scramble config */
        if (cfg->phase_scramble_on) {
                fmcw_radio_switch_bank(radio, 3);
                RADIO_WRITE_BANK_REG(3, FMCW_PS_TAP0, REG_L(cfg->phase_scramble_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_PS_TAP1, REG_M(cfg->phase_scramble_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_PS_TAP2, REG_H(cfg->phase_scramble_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_PS_TAP3, REG_INT(cfg->phase_scramble_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE0, REG_L(cfg->phase_scramble_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE1, REG_M(cfg->phase_scramble_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE2, REG_H(cfg->phase_scramble_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE3, REG_INT(cfg->phase_scramble_init_state));

                if(cfg->phase_scramble_on & 0x2){
                        RADIO_WRITE_BANK_REG(3, PS_RM, 0);
                }
                /* Phase scramble configureation for group 1 */
                fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
                fmcw_radio_reg_write(radio, R5_FMCW_PS_EN_1, 0x1);  /*enable phase scramble*/
         }

        /* frequency hopping config */
        if (cfg->freq_hopping_on) {
                /*config XOR chain*/
                fmcw_radio_switch_bank(radio, 3);
                RADIO_WRITE_BANK_REG(3, FMCW_HP_TAP0, REG_L(cfg->freq_hopping_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_HP_TAP1, REG_M(cfg->freq_hopping_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_HP_TAP2, REG_H(cfg->freq_hopping_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_HP_TAP3, REG_INT(cfg->freq_hopping_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_HP_STATE0, REG_L(cfg->freq_hopping_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_HP_STATE1, REG_M(cfg->freq_hopping_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_HP_STATE2, REG_H(cfg->freq_hopping_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_HP_STATE3, REG_INT(cfg->freq_hopping_init_state));

                if(cfg->freq_hopping_on & 0x2){
                        RADIO_WRITE_BANK_REG(3, HP_RM, 0);
                }

                /*config freq parameters*/
                fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
                fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_HP_1_0, REG_L(radio->hp_start_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_HP_1_1, REG_M(radio->hp_start_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_HP_1_2, REG_H(radio->hp_start_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_HP_1_3, REG_INT(radio->hp_start_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_HP_1_0, REG_L(radio->hp_stop_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_HP_1_1, REG_M(radio->hp_stop_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_HP_1_2, REG_H(radio->hp_stop_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_HP_1_3, REG_INT(radio->hp_stop_freq));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_HP_1_0, REG_L(radio->step_up));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_HP_1_1, REG_M(radio->step_up));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_HP_1_2, REG_H(radio->step_up));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_HP_1_3, REG_INT(radio->step_up));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_HP_1_0, REG_L(radio->step_down));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_HP_1_1, REG_M(radio->step_down));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_HP_1_2, REG_H(radio->step_down));
                fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_HP_1_3, REG_INT(radio->step_down));
                fmcw_radio_reg_write(radio, R5_FMCW_IDLE_HP_1_0, REG_L(radio->cnt_wait));
                fmcw_radio_reg_write(radio, R5_FMCW_IDLE_HP_1_1, REG_M(radio->cnt_wait));
                fmcw_radio_reg_write(radio, R5_FMCW_IDLE_HP_1_2, REG_H(radio->cnt_wait));
                fmcw_radio_reg_write(radio, R5_FMCW_IDLE_HP_1_3, REG_INT(radio->cnt_wait));
                fmcw_radio_reg_write(radio, R5_FMCW_HP_EN_1, 0x1);
        }

        /* anti velocity deambiguity config */
        if (cfg->anti_velamb_en == true) {
                fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
                fmcw_radio_reg_write(radio,R5_FMCW_CS_AVA_DLY_1_0, REG_L(radio->anti_velamb_cycle));
                fmcw_radio_reg_write(radio,R5_FMCW_CS_AVA_DLY_1_1, REG_M(radio->anti_velamb_cycle));
                fmcw_radio_reg_write(radio,R5_FMCW_CS_AVA_DLY_1_2, REG_H(radio->anti_velamb_cycle));
                fmcw_radio_reg_write(radio,R5_FMCW_CS_AVA_DLY_1_3, REG_INT(radio->anti_velamb_cycle));

                /* configure extra chirp */
                for (int ch = 0; ch < MAX_NUM_TX; ch++) {
                        if (cfg->tx_groups[ch] & 0xF) {
                                fmcw_radio_reg_write(radio, R5_FMCW_CS_AVA_EN_1, ( ch << 2 ) | 0x1);
                                break;
                        }
                }
        } else {
                fmcw_radio_reg_write(radio,R5_FMCW_CS_AVA_EN_1, 0x0);
        }

        /* chirp shifting config */
        if (cfg->chirp_shifting_on) {
                fmcw_radio_switch_bank(radio, 3);
                RADIO_WRITE_BANK_REG(3, FMCW_CS_TAP0, REG_L(cfg->chirp_shifting_init_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_CS_TAP1, REG_M(cfg->chirp_shifting_init_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_CS_TAP2, REG_H(cfg->chirp_shifting_init_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_CS_TAP3, REG_INT(cfg->chirp_shifting_init_tap));
                RADIO_WRITE_BANK_REG(3, FMCW_CS_STATE0, REG_L(cfg->chirp_shifting_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_CS_STATE1, REG_M(cfg->chirp_shifting_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_CS_STATE2, REG_H(cfg->chirp_shifting_init_state));
                RADIO_WRITE_BANK_REG(3, FMCW_CS_STATE3, REG_INT(cfg->chirp_shifting_init_state));

                if(cfg->chirp_shifting_on & 0x2){
                        RADIO_WRITE_BANK_REG(3, CS_RM, 0);
                }

                fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
                fmcw_radio_reg_write(radio, R5_FMCW_CS_DLY_1_0, REG_L(radio->chirp_shifting_cyle));
                fmcw_radio_reg_write(radio, R5_FMCW_CS_DLY_1_1, REG_M(radio->chirp_shifting_cyle));
                fmcw_radio_reg_write(radio, R5_FMCW_CS_DLY_1_2, REG_H(radio->chirp_shifting_cyle));
                fmcw_radio_reg_write(radio, R5_FMCW_CS_DLY_1_3, REG_INT(radio->chirp_shifting_cyle));
                fmcw_radio_reg_write(radio, R5_FMCW_CS_EN_1, 0x1);
        }

        if(cfg->agc_mode)
        {
            fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
            RADIO_MOD_BANK_REG(5,FMCW_AGC_EN_1,AGC_EN_1,1);   //agc enable
        }
#if DDM_EN        /* Doppler division MIMO */
        fmcw_radio_DDM_cmd_cfg(radio);
#endif

        fmcw_radio_switch_bank(radio, 3);
        //MODE_SEL
        //0x00 hold

        //0x01 realtime
        //0x02 predefined
        //0x05 bypass_SDM
        RADIO_WRITE_BANK_REG(3, FMCW_MODE_SEL, 0x2);
        /* Only need to reset once at beginning */
        RADIO_MOD_BANK_REG(3, FMCW_START, ADDER_RSTN, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, ADDER_RSTN, 0x1);
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_SYNC, EN, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x1);
        RADIO_MOD_BANK_REG(3, FMCW_SYNC, SEL, 0x3);
        RADIO_MOD_BANK_REG(3, FMCW_SYNC_DLY, SYNC_DLY_EN, 0x1);
        RADIO_MOD_BANK_REG(3, FMCW_SYNC, EN, 0x1);
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x0);
        fmcw_radio_switch_bank(radio, old_bank);

        return result;
}

void fmcw_radio_start_fmcw(fmcw_radio_t *radio)
{
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;

        /*Configure all Txs Phase Shifter */
        uint32_t ch, reg_val;
        for (ch = 0; ch < MAX_NUM_TX; ch++)
        {
            reg_val = phase_val_2_reg_val(cfg->tx_phase_value[ch]);
            fmcw_radio_set_tx_phase(radio, ch, reg_val);
        }

        /*refresh radio CMD when anti-vel is enabled*/
        if (cfg->anti_velamb_en){
                if((cfg->nvarray >= 3) && cfg->agc_mode)
                {

                }
                else
                {
                        fmcw_radio_cmd_cfg(radio, true);
                }

        }
        /* set txlo buffer by tx groups status */
        fmcw_radio_txlobuf_on(radio);

        if (cfg->nvarray >= 2){
                int ch;
                /* switch SPI source */
                radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
                if (cfg->bpm_mode==true) {
                        for (ch = 0; ch < MAX_NUM_TX; ch++) {
                                if (cfg->tx_groups[ch] > 0){
                                        fmcw_radio_set_tx_status(radio, ch, 0xf);
                                }
                        }
                } else {
                        for (ch = 0; ch < MAX_NUM_TX; ch++){
                                //fmcw_radio_set_tx_status(radio, ch, 0x0);
                                fmcw_radio_set_tx_status(radio, ch, 0xF);
                        }
                }
        }
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        /* Reset before start */
        RADIO_WRITE_BANK_REG(3, FMCW_MODE_SEL, 0x2);
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x0);

        /* DMU triggers FMCW_START */
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SEL, 1);
        fmcw_radio_switch_bank(radio, old_bank);
        /* Enable CMD */
        raw_writel(RADIO_SPI_CMD_SRC_SEL, RADIO_SPI_CMD_FMCW  );
        raw_writel(REG_DMU_FMCW_START + REL_REGBASE_DMU, 1);
}

void fmcw_radio_stop_fmcw(fmcw_radio_t *radio)
{
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        if (cfg->nvarray >= 2 || cfg->phase_scramble_on == true ){
                radio_spi_cmd_mode(RADIO_SPI_CMD_CPU);
        }
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        RADIO_MOD_BANK_REG(3,FMCW_START,START_SPI,0x0);
        fmcw_radio_switch_bank(radio, old_bank);
}

/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 * data          : RX_TIA_133omhs:0xF / RX_TIA_250ohms:0x1 / RX_TIA_500ohms:0x2 / RX_TIA_1000ohms:0x4 / RX_TIA_2000ohms:0x8
 */
void fmcw_radio_set_tia_gain(fmcw_radio_t *radio,
                             int32_t channel_index, int32_t gain)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        int ch;
        if (channel_index != -1){
                RADIO_MOD_BANK_CH_REG(0, channel_index, RX_TN0, TIA_RFB_SEL, gain);
        }
        else {
                for(ch = 0; ch < MAX_NUM_RX; ch++){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, gain);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_get_tia_gain(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        char rx_gain = RADIO_READ_BANK_CH_REG_FIELD(0, channel_index, RX_TN0, TIA_RFB_SEL);
        fmcw_radio_switch_bank(radio, old_bank);
        return rx_gain;
}

/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 * data          : RX_VGA1_6dB:0x01 / RX_VGA1_9dB:0x02 / RX_VGA1_12dB:0x03 /
 *                 RX_VGA1_15dB:0x04 / RX_VGA1_18dB:0x05 / RX_VGA1_21dB:0x06
 */
void fmcw_radio_set_vga1_gain(fmcw_radio_t *radio, int32_t channel_index, int32_t gain)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        int ch;
        if (channel_index != -1){
                RADIO_MOD_BANK_CH_REG(0, channel_index, RX_TN1, VGA1_GAINSEL, gain);
        }
        else {
                for(ch = 0; ch < MAX_NUM_RX; ch++){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, gain);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_get_vga1_gain(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        char rx_gain = RADIO_READ_BANK_CH_REG_FIELD(0, channel_index, RX_TN1, VGA1_GAINSEL);
        fmcw_radio_switch_bank(radio, old_bank);
        return rx_gain;
}

/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 * data          : RX_VGA1_5dB:0x01 / RX_VGA1_8dB:0x02 / RX_VGA1_11dB:0x03 /
 *                 RX_VGA1_14dB:0x04 / RX_VGA1_16dB:0x05 / RX_VGA1_19dB:0x06
 */
void fmcw_radio_set_vga2_gain(fmcw_radio_t *radio, int32_t channel_index, char gain)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        int ch;
        if (channel_index != -1){
                RADIO_MOD_BANK_CH_REG(0, channel_index, RX_TN1, VGA2_GAINSEL, gain);
        }
        else {
                for(ch = 0; ch < MAX_NUM_RX; ch++){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, gain);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_get_vga2_gain(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        char rx_gain = RADIO_READ_BANK_CH_REG_FIELD(0, channel_index, RX_TN1, VGA2_GAINSEL);
        fmcw_radio_switch_bank(radio, old_bank);
        return rx_gain;
}

/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 * data          : TX_ON:0xF / TX_OFF:0x0
 */
void fmcw_radio_set_tx_status(fmcw_radio_t *radio, int32_t channel_index, char status)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        int ch;
        if (channel_index != -1){
                //fix glitch bug 1128
                if (cfg->anti_velamb_en){
                        if((cfg->nvarray >= 3) && cfg->agc_mode)
                        {

                        }
                        else
                        {
                                status = (cfg->tx_groups[channel_index] == 1) ? 0x0f : 0x00 ;
                        }
                        RADIO_WRITE_BANK_CH_REG(1, channel_index, TX_EN0, status);
                }
                else{
                        RADIO_WRITE_BANK_CH_REG(1, channel_index, TX_EN0, status);
                }
        }
        else {
                for(ch = 0; ch < MAX_NUM_TX; ch++){
                        RADIO_WRITE_BANK_CH_REG(1, ch, TX_EN0, status);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_get_tx_status(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank;
        char tx_status;
        old_bank = fmcw_radio_switch_bank(radio, 1);
        tx_status = RADIO_READ_BANK_CH_REG(1, channel_index, TX_EN0);
        fmcw_radio_switch_bank(radio, old_bank);
        return tx_status;
}

/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 * data          : TX_POWER_DEFAULT:0xAA / TX_POWER_MAX:0xFF / TX_POWER_MAX_SUB5:0x88 / TX_POWER_MAX_SUB10:0x0
 */
//TO-DO: added power_index_0 and power_index_1
void fmcw_radio_set_tx_power(fmcw_radio_t *radio,
                             int32_t channel_index, char power_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        int ch;
        if (channel_index != -1){
                RADIO_WRITE_BANK_CH_REG(1, channel_index, TX_BI0, power_index);
                RADIO_WRITE_BANK_CH_REG(1, channel_index, TX_BI1, power_index);
        }
        else {
                for(ch = 0; ch < MAX_NUM_TX; ch++){
                        RADIO_WRITE_BANK_CH_REG(1, ch, TX_BI0, power_index);
                        RADIO_WRITE_BANK_CH_REG(1, ch, TX_BI1, power_index);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

//TO-DO : added for TX_BI1 ???
int32_t fmcw_radio_get_tx_power(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        char tx_power = RADIO_READ_BANK_CH_REG(1, channel_index, TX_BI0);
        fmcw_radio_switch_bank(radio, old_bank);
        return tx_power;
}

/*
 * channel_index : CH0 / CH0 / CH1 / CH2 / CH_ALL
 *      #define TX_PHASE_0       0x0f0fu
 *      #define TX_PHASE_45      0x000fu
 *      #define TX_PHASE_90      0x1f0fu
 *      #define TX_PHASE_135     0x1f00u
 *      #define TX_PHASE_180     0x1f1fu
 *      #define TX_PHASE_225     0x001fu
 *      #define TX_PHASE_270     0x0f1fu
 *      #define TX_PHASE_315     0x0f00u
 */
uint32_t phase_val_2_reg_val(uint32_t phase_val)
{
        uint32_t reg_val = 0x0;
        switch (phase_val) {
        case 0:
                reg_val = TX_PHASE_0;
                break;
        case 45:
                reg_val = TX_PHASE_45;
                break;
        case 90:
                reg_val = TX_PHASE_90;
                break;
        case 135:
                reg_val = TX_PHASE_135;
                break;
        case 180:
                reg_val = TX_PHASE_180;
                break;
        case 225:
                reg_val = TX_PHASE_225;
                break;
        case 270:
                reg_val = TX_PHASE_270;
                break;
        case 315:
                reg_val = TX_PHASE_315;
                break;
        default:
                reg_val = 0x0;
                break;
        }
        return reg_val;
}
uint32_t reg_val_2_phase_val(uint32_t reg_val)
{
        uint32_t phase_val = 0xfff;
        switch (reg_val) {
        case TX_PHASE_0:
                phase_val = 0;
                break;
        case TX_PHASE_45:
                phase_val = 45;
                break;
        case TX_PHASE_90:
                phase_val = 90;
                break;
        case TX_PHASE_135:
                phase_val = 135;
                break;
        case TX_PHASE_180:
                phase_val = 180;
                break;
        case TX_PHASE_225:
                phase_val = 225;
                break;
        case TX_PHASE_270:
                phase_val = 270;
                break;
        case TX_PHASE_315:
                phase_val = 315;
                break;
        default:
                phase_val = 0xfff;
                break;
        }
        return phase_val;
}

void fmcw_radio_set_tx_phase(fmcw_radio_t *radio, int32_t channel_index, int32_t phase_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        char tx_phase_i = (char) (phase_index >> 8);
        char tx_phase_q = (char) phase_index;
        int ch;
        if (channel_index != -1) {
                RADIO_WRITE_BANK_CH_REG(1, channel_index, TX_TN0, tx_phase_i);
                RADIO_WRITE_BANK_CH_REG(1, channel_index, TX_TN1, tx_phase_q);
        } else {
                for(ch = 0; ch < MAX_NUM_TX; ch++) {
                        RADIO_WRITE_BANK_CH_REG(1, ch, TX_TN0, tx_phase_i);
                        RADIO_WRITE_BANK_CH_REG(1, ch, TX_TN1, tx_phase_q);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_get_tx_phase(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        int32_t tx_phase_index = RADIO_READ_BANK_CH_REG(1, channel_index, TX_TN0) << 8 |
                RADIO_READ_BANK_CH_REG(1, channel_index, TX_TN1);
        fmcw_radio_switch_bank(radio, old_bank);
        return tx_phase_index;
}

//No need to judge if read is greater than 128 or not
float fmcw_radio_get_temperature(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        /* Enable 800M ADC CLK */
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
        /* Enable safety monitor LDO */
        RADIO_MOD_BANK_REG(0, LDO25_SM, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO11_SM, LDO_EN, 0x1);
#if defined(FUNC_SAFETY) && (SAFETY_FEATURE_SM2 == 1)
        /* Clear IRQ of LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
        /* Disable AVDD33 Monitor */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_RDAC_EN, 0x0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_EN, 0x0);
#endif
        /* Enable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x1);
        /* AUXADC2 on and reset */
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_BUF_BP, 0x0);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_BUF_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_BI_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_OP1_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_OP2_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_REFGEN_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_VCMGEN_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_RST, 0x1);

        /* select AUXADC2 InputMUX */
        RADIO_MOD_BANK_REG(1, DTSMD2_MUXIN_SEL, TS_BG_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSMD2_MUXIN_SEL, TS_VPTAT_CAL, 0x4);
        RADIO_MOD_BANK_REG(1, DTSMD2_MUXIN_SEL, DTSDM_MUXIN_SEL, 0x0);

        /* AUXADC2 de-reset */
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_RST, 0x0);

        /* AUXADC2 Filter de-reset */
        fmcw_radio_switch_bank(radio, 2);
        RADIO_WRITE_BANK_REG(2, DC_FILTER2_RST_EN, 0x01);
        MDELAY(2);

        /* Disable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        /* Disabel AUXADC2 */
        RADIO_WRITE_BANK_REG(1, DTSDM2_EN, 0x0);

        /* read back AUXADC2 Filter Output Digital Bits */
        uint8_t doutL, doutM, doutH;
        float dout, radio_temp;
        doutL =  RADIO_READ_BANK_REG(1, DTSDM2_DAT0);
        doutM =  RADIO_READ_BANK_REG(1, DTSDM2_DAT1);
        doutH =  RADIO_READ_BANK_REG(1, DTSDM2_DAT2);
        dout = doutL + ( doutM << 8 ) + ( doutH << 16 );

        /* return voltage measurement, formula refer to 125C */
        radio_temp = ((float)(dout) - ts_coefficient[T_SENSOR_D])/ts_coefficient[T_SENSOR_K] + ts_coefficient[T_SENSOR_T];

#if defined(FUNC_SAFETY) && (SAFETY_FEATURE_SM2 == 1)
        fmcw_radio_sm_avdd33_monitor_IRQ(radio, false);
#endif
        fmcw_radio_switch_bank(radio, old_bank);
        return radio_temp;
}

/* gain_compensation is used under high ambient temperatures */
#if HTOL_T == 1
void fmcw_radio_gain_compensation(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        int ch;
        RADIO_MOD_BANK_REG(0, LO_LDO0, LDO11_LMP_VO_SEL, 0x6);
        RADIO_MOD_BANK_REG(0, LO_LDO1, LDO11_LMO_VO_SEL, 0x7);
        RADIO_MOD_BANK_REG(0, LO_LDO2, LDO11_TXLO_VO_SEL, 0x6);
        RADIO_MOD_BANK_REG(0, LO_LDO3, LDO11_RXLO_VO_SEL, 0x7);
        RADIO_MOD_BANK_REG(0, RX_LDO0, LDO11_RFN_VO_SEL, 0x5);
        RADIO_MOD_BANK_REG(0, RX_LDO1, LDO11_RFS_VO_SEL, 0x5);
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, TX_LDO0, LDO11_TX0_VO_SEL, 0x5);
        RADIO_MOD_BANK_REG(1, TX_LDO1, LDO11_TX1_VO_SEL, 0x5);
        RADIO_MOD_BANK_REG(1, TX_LDO2, LDO11_TX2_VO_SEL, 0x5);
        RADIO_MOD_BANK_REG(1, TX_LDO3, LDO11_TX3_VO_SEL, 0x5);
        for(ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_CH_REG(1, ch, PA_LDO, LDO11_TX_PA_VO_SEL, 0xd);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}
#endif /* gain_compensation */

void fmcw_radio_power_on(fmcw_radio_t *radio)
{
        fmcw_radio_ldo_on(radio);
        fmcw_radio_pll_on(radio);
        fmcw_radio_lo_on(radio, true);
        /*set TX power as default:0xAA */
        fmcw_radio_set_tx_power(radio, -1, TX_POWER_DEFAULT);
        fmcw_radio_tx_on(radio);
        fmcw_radio_rx_on(radio, true);
        fmcw_radio_rc_calibration(radio);
        fmcw_radio_adc_cmp_calibration(radio);
        fmcw_radio_adc_on(radio);
        fmcw_radio_auxadc_trim(radio);
        fmcw_radio_temp_sensor_trim(radio);
}

void fmcw_radio_power_off(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        /* FP disable */
        RADIO_MOD_BANK_REG(0, FP_EN, TSPCDIV_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_EN, CMLDIV_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_EN, VC_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_EN, CP_EN, 0x0);
        fmcw_radio_switch_bank(radio, old_bank);
}

bool fmcw_radio_is_running(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        uint8_t mode = RADIO_READ_BANK_REG(3, FMCW_MODE_SEL);
        uint8_t start = RADIO_READ_BANK_REG_FIELD(3, FMCW_START, START_SPI);
        fmcw_radio_switch_bank(radio, 5  + radio->frame_type_id);
        uint8_t chirp_size_1_0 = RADIO_READ_BANK_REG(5, FMCW_CHIRP_SIZE_1_0);
        uint8_t chirp_size_1_1 = RADIO_READ_BANK_REG(5, FMCW_CHIRP_SIZE_1_1);
        fmcw_radio_switch_bank(radio, old_bank);
        bool ret = false;
        if (mode == 0x2){
                ret = (start == 1 && !chirp_size_1_0 && !chirp_size_1_1 ) ? true : false;
        }
        else if (mode == 0x3){
                ret = false;
        }
        else{
                EMBARC_PRINTF("\n\n");
        }
        return ret;
}

int32_t fmcw_radio_vctrl_on(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        //enable PLL's local switch
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_T_SW_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_T_EN, 0x1);
        //enable TP_ANA1 and mux VCTRL to output
        RADIO_MOD_BANK_REG(0, TPANA1, EN, 0x1);
        RADIO_MOD_BANK_REG(0, TPANA1, T_MUX_1_EN, 0x1);
        RADIO_MOD_BANK_REG(0, TPANA1, T_MUX_1_SEL, 0x38);
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

int32_t fmcw_radio_vctrl_off(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        //disable PLL's local switch
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_T_SW_EN, 0x0);
        RADIO_MOD_BANK_REG(0, FP_LF, VTR_T_EN, 0x0);
        //disable TP_ANA1
        RADIO_MOD_BANK_REG(0, TPANA1, EN, 0x0);
        RADIO_MOD_BANK_REG(0, TPANA1, T_MUX_1_EN, 0x0);
        RADIO_MOD_BANK_REG(0, TPANA1, T_MUX_1_SEL, 0x00);
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 */
void fmcw_radio_if_output_on(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);
        int ch;
        RADIO_MOD_BANK_REG(2, LVDS_LDO25, LDO25_LVDS_LDO_EN, 0x1);
        if (channel_index != -1) {
                RADIO_MOD_BANK_CH_REG(2, channel_index, BUFFER, EN, 0x1);
        } else {
                for(ch = 0; ch < MAX_NUM_RX; ch++){
                        RADIO_MOD_BANK_CH_REG(2, ch, BUFFER, EN, 0x1);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

/* enable buffer will output IF to output pad, only used for debug */
void fmcw_radio_if_output_off(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);
        int ch;
        if (channel_index != -1) {
                RADIO_MOD_BANK_CH_REG(2, channel_index, BUFFER, EN, 0x0);
        } else {
                RADIO_MOD_BANK_REG(2, LVDS_LDO25, LDO25_LVDS_LDO_EN, 0x0);
                for(ch = 0; ch < MAX_NUM_RX; ch++){
                        RADIO_MOD_BANK_CH_REG(2, ch, BUFFER, EN, 0x0);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

//control TX's LDO and PA's LDO
void fmcw_radio_tx_ch_on(fmcw_radio_t *radio, int32_t channel_index, bool enable)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        uint8_t enable_4bit;
// #ifndef FUNC_SAFETY
        uint8_t enable_8bit;
// #endif

        if (enable == true) {
                enable_4bit = 0x0f;
// #ifndef FUNC_SAFETY
                enable_8bit = 0xff;
// #endif
        } else {
                enable_4bit = 0x00;
// #ifndef FUNC_SAFETY
                enable_8bit = 0x00;
// #endif
        }
        if (channel_index == -1) {
// #ifndef FUNC_SAFETY
                RADIO_WRITE_BANK_REG(1, TX_LDO_EN, enable_8bit);
// #endif
                for(uint8_t ch = 0; ch < MAX_NUM_TX; ch++){
                        RADIO_WRITE_BANK_CH_REG(1, ch, TX_EN0, enable_4bit);
                //Wait to add after VAM is done
                }
        } else if (channel_index == 0) {
// #ifndef FUNC_SAFETY
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX0_EN, enable);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX0_PA_EN, enable);
// #endif
                RADIO_WRITE_BANK_REG(1, CH0_TX_EN0, enable_4bit);
        } else if (channel_index == 1) {
// #ifndef FUNC_SAFETY
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX1_EN, enable);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX1_PA_EN, enable);
// #endif
                RADIO_WRITE_BANK_REG(1, CH1_TX_EN0, enable_4bit);
        } else if (channel_index == 2) {
// #ifndef FUNC_SAFETY
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX2_EN, enable);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX2_PA_EN, enable);
// #endif
                RADIO_WRITE_BANK_REG(1, CH2_TX_EN0, enable_4bit);
        } else if (channel_index == 3) {
// #ifndef FUNC_SAFETY
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX3_EN, enable);
                RADIO_MOD_BANK_REG(1, TX_LDO_EN, LDO11_TX3_PA_EN, enable);
// #endif
                RADIO_WRITE_BANK_REG(1, CH3_TX_EN0, enable_4bit);
        } else {
                EMBARC_PRINTF("\n\n");
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

static void fmcw_radio_tx_restore_proc(fmcw_radio_t *radio)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        bool enable;
        for(uint8_t ch = 0; ch < MAX_NUM_TX; ch++) {
                enable = !(cfg->tx_groups[ch] == 0);
                fmcw_radio_tx_ch_on(NULL, ch, enable);
        }
}

void fmcw_radio_tx_restore(fmcw_radio_t *radio)
{
#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER)
                fmcw_radio_tx_restore_proc(radio);
#else
                fmcw_radio_tx_restore_proc(radio);
#endif
}

void fmcw_radio_loop_test_en(fmcw_radio_t *radio, bool enable)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 9);
        int val = (enable == true)?1:0;
        RADIO_WRITE_BANK_REG(9, LP_TST_EN, val);
        /* reset SDM */
        fmcw_radio_switch_bank(radio, 3);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x1);
        /* In loop test, rstn_fmcw_gen must be 0 */
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, !val);
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_dac_reg_cfg_outer(fmcw_radio_t *radio) /* outer circle */
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 9);
        int ch;
        /* turn on dac */
        RADIO_WRITE_BANK_REG(9, LP_TST_EN1, 0x7F);
        RADIO_WRITE_BANK_REG(9, LP_TST_EN2, 0xBF);
        RADIO_WRITE_BANK_REG(9, LP_TST_EN3, 0x00);
        RADIO_WRITE_BANK_REG(9, DAC_LP_TST, 0x11);
        for(ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_WRITE_BANK_CH_REG(9, ch, RXBB, 0x01);
        }
        fmcw_radio_switch_bank(radio, 2);
        RADIO_MOD_BANK_REG(2, BIST_EN1, BIST_EN_SP, 0x0);
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_dac_reg_restore(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);
        uint8_t ch; /* channel index*/
        RADIO_WRITE_BANK_REG(2, BIST_LDO, 0x40);
        RADIO_WRITE_BANK_REG(2, BIST_EN0, 0x00);
        RADIO_WRITE_BANK_REG(2, BIST_EN1, 0x00);
        RADIO_WRITE_BANK_REG(2, DAC_EN, 0x10);
        fmcw_radio_switch_bank(radio, 0);
        for(ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_WRITE_BANK_CH_REG(0, ch, RX_T, 0x1);
                RADIO_WRITE_BANK_CH_REG(0, ch, RX_BI0, 0x8);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_dac_reg_cfg_inner(fmcw_radio_t *radio, uint8_t inject_num, uint8_t out_num) /* inner circle */
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 9);
        uint8_t ch; /* channel index*/
        /* turn on dac */
        RADIO_WRITE_BANK_REG(9, LP_TST_EN1, 0x70);
        RADIO_WRITE_BANK_REG(9, LP_TST_EN2, 0xC0);
        RADIO_WRITE_BANK_REG(9, DAC_LP_TST, 0x11);

        for(ch = 0; ch < MAX_NUM_RX; ch++) {
                /* inject choice */
                switch (inject_num) {
                case 0: /* TIA injected*/
                        RADIO_WRITE_BANK_REG(9, LP_TST_EN3, 0xFF);
                        break;
                case 1: /* HPF1 injected*/
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, TSTMUX_SEL_LP_TST, 0x8);
                        break;
                case 2: /* VGA1 injected*/
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, TSTMUX_SEL_LP_TST, 0x4);
                        break;
                case 3: /* HPF2 injected*/
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, TSTMUX_SEL_LP_TST, 0x2);
                        break;
                case 4: /* VGA2 injected*/
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, TSTMUX_SEL_LP_TST, 0x1);
                        break;
                default: /* HPF1 injected*/
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, TSTMUX_SEL_LP_TST, 0x8);
                        break;
                }

                /* out choice */
                switch (out_num) {
                case 0: /* TIA out */
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, OMUX_SEL_LP_TST, 0x8);
                        break;
                case 1: /* HPF1 out */
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, OMUX_SEL_LP_TST, 0x4);
                        break;
                case 2: /* VGA1 out */
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, OMUX_SEL_LP_TST, 0x2);
                        break;
                case 3: /* VGA2 out */
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, OMUX_SEL_LP_TST, 0x1);
                        break;
                default: /* VGA2 out */
                        RADIO_MOD_BANK_CH_REG(9, ch, RXBB, OMUX_SEL_LP_TST, 0x1);
                        break;
                }
        }

        fmcw_radio_switch_bank(radio, 2);
        RADIO_MOD_BANK_REG(2, BIST_EN1, BIST_EN_SP, 0x1);
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_dc_reg_cfg(fmcw_radio_t *radio, int32_t channel_index, int16_t dc_offset, bool dc_calib_print_ena)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);
        int16_t dc_offset_reverse;
        uint8_t dc_high8 = 0;
        uint8_t dc_low8  = 0;
        uint8_t old_dc_high8 = 0;
        uint8_t old_dc_low8  = 0;
        //readback bandwidth sel from chip
        uint8_t bdw = RADIO_READ_BANK_REG_FIELD(2,CH1_ADC_FILTER,BDW_SEL);
        /* change to 2 values */
        /* write FMCW registers(15 bits), but the MSB 3 bits are all sign bits*/
        /* dc_low8 using 8 bits, dc_high8 using 7 bits */
        if ( bdw == 0 ){
                dc_offset_reverse = (-dc_offset) / 4; /* adc data has left-shift 2 bits in RTL, so / 4*/
        }
        else{
                dc_offset_reverse = (-dc_offset) / 8; /* adc data has left-shift 1 bits in RTL, so / 2*/
        }

        dc_high8 = REG_H7(dc_offset_reverse); /* mask the high 7 bits*/
        dc_low8  = REG_L(dc_offset_reverse);  /* mask the low  8 bits*/

        if (0 == radio->frame_type_id) { // Store dc_offset value to default common register
                /* read */
                old_dc_low8  = RADIO_READ_BANK_CH_REG(2, channel_index, FILTER_DC_CANCEL_1);
                old_dc_high8 = RADIO_READ_BANK_CH_REG(2, channel_index, FILTER_DC_CANCEL_2);
                /* write */
                RADIO_WRITE_BANK_CH_REG(2, channel_index, FILTER_DC_CANCEL_1, dc_low8);
                RADIO_WRITE_BANK_CH_REG(2, channel_index, FILTER_DC_CANCEL_2, dc_high8);
        }
        // Store dc_offset value to banked registers which located in radio bank 5, 6, 7, 8
        uint8_t old_bank1 = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        /* read */
        // The different banked registers share the same address values, so choose bank5 address values
        old_dc_low8  = RADIO_READ_BANK_CH_REG(5, channel_index, FILTER_DC_CANCEL_1_1);
        old_dc_high8 = RADIO_READ_BANK_CH_REG(5, channel_index, FILTER_DC_CANCEL_1_2);
        /* write */
        RADIO_WRITE_BANK_CH_REG(5, channel_index, FILTER_DC_CANCEL_1_1, dc_low8);
        RADIO_WRITE_BANK_CH_REG(5, channel_index, FILTER_DC_CANCEL_1_2, dc_high8);
        fmcw_radio_switch_bank(radio, old_bank1);
        /* check */
        if(dc_calib_print_ena == 1){
                EMBARC_PRINTF("frame type %d: channel %d before modification, old_dc_high8 = 0x%x, old_dc_low8 = 0x%x\n", radio->frame_type_id, channel_index, old_dc_high8, old_dc_low8);
                EMBARC_PRINTF("frame type %d: channel %d after modification,  new_dc_high8 = 0x%x, new_dc_low8 = 0x%x\n", radio->frame_type_id, channel_index, dc_high8, dc_low8);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_adc_cmp_calibration(fmcw_radio_t *radio)
{
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Enable 400M ADC CLK and ADC Local CLK*/
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_400M_ADC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_SDMADC_CLKSEL, 0x0);
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, ADC_MUX_O_SEL, CLK_EN, 0x1);

        /* LDO ON */
        RADIO_MOD_BANK_REG(1, ADC_LDO0, LDO11_ADC12_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO1, LDO12_ADC12_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO2, LDO25_ADC12_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO3, LDO11_ADC34_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO4, LDO12_ADC34_EN, 0x1);
        RADIO_MOD_BANK_REG(1, ADC_LDO5, LDO25_ADC34_EN, 0x1);
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, RX_LDO0, LDO11_RFN_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RX_LDO1, LDO11_RFS_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RX_LDO2, LDO25_BBN_EN, 0x1);
        RADIO_MOD_BANK_REG(0, RX_LDO3, LDO25_BBS_EN, 0x1);

        /* RXBB outmux de-selsection */
        RADIO_MOD_BANK_REG(0,CH0_RX_T,OMUX_SEL,0x0);
        RADIO_MOD_BANK_REG(0,CH1_RX_T,OMUX_SEL,0x0);
        RADIO_MOD_BANK_REG(0,CH2_RX_T,OMUX_SEL,0x0);
        RADIO_MOD_BANK_REG(0,CH3_RX_T,OMUX_SEL,0x0);

        /* Enable ADC and Reset */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_WRITE_BANK_REG(1, CH0_ADC_EN0, 0x7F);
        RADIO_WRITE_BANK_REG(1, CH0_ADC_EN1, 0xCE);
        RADIO_WRITE_BANK_REG(1, CH1_ADC_EN0, 0x7F);
        RADIO_WRITE_BANK_REG(1, CH1_ADC_EN1, 0xCE);
        RADIO_WRITE_BANK_REG(1, CH2_ADC_EN0, 0x7F);
        RADIO_WRITE_BANK_REG(1, CH2_ADC_EN1, 0xCE);
        RADIO_WRITE_BANK_REG(1, CH3_ADC_EN0, 0x7F);
        RADIO_WRITE_BANK_REG(1, CH3_ADC_EN1, 0xCE);

        /* ADC CMP Calibration Pre-Setting */
        RADIO_WRITE_BANK_REG(1, CH0_ADC_TN10, 0x29);
        RADIO_WRITE_BANK_REG(1, CH0_ADC_TN11, 0x60);
        RADIO_WRITE_BANK_REG(1, CH1_ADC_TN10, 0x29);
        RADIO_WRITE_BANK_REG(1, CH1_ADC_TN11, 0x60);
        RADIO_WRITE_BANK_REG(1, CH2_ADC_TN10, 0x29);
        RADIO_WRITE_BANK_REG(1, CH2_ADC_TN11, 0x60);
        RADIO_WRITE_BANK_REG(1, CH3_ADC_TN10, 0x29);
        RADIO_WRITE_BANK_REG(1, CH3_ADC_TN11, 0x60);
        /* Wait for 1ms */
        MDELAY(1);

        /* ADC CMP Calibration and waiting for allready=1
           due to corner&temperature, need at most 1ms) */
        RADIO_MOD_BANK_REG(1, CH0_ADC_TN11, CMP_OSCALIB_START, 0x1);
        RADIO_MOD_BANK_REG(1, CH1_ADC_TN11, CMP_OSCALIB_START, 0x1);
        RADIO_MOD_BANK_REG(1, CH2_ADC_TN11, CMP_OSCALIB_START, 0x1);
        RADIO_MOD_BANK_REG(1, CH3_ADC_TN11, CMP_OSCALIB_START, 0x1);
        MDELAY(1);

        /* ADC CMP Calibration Post-Setting */
        RADIO_MOD_BANK_REG(1, CH0_ADC_TN10, SATDET_EN, 0x0);
        RADIO_MOD_BANK_REG(1, CH0_ADC_TN11, CMP_OSCALIB_SHORT2VCM_EN, 0x0);
        RADIO_MOD_BANK_REG(1, CH1_ADC_TN10, SATDET_EN, 0x0);
        RADIO_MOD_BANK_REG(1, CH1_ADC_TN11, CMP_OSCALIB_SHORT2VCM_EN, 0x0);
        RADIO_MOD_BANK_REG(1, CH2_ADC_TN10, SATDET_EN, 0x0);
        RADIO_MOD_BANK_REG(1, CH2_ADC_TN11, CMP_OSCALIB_SHORT2VCM_EN, 0x0);
        RADIO_MOD_BANK_REG(1, CH3_ADC_TN10, SATDET_EN, 0x0);
        RADIO_MOD_BANK_REG(1, CH3_ADC_TN11, CMP_OSCALIB_SHORT2VCM_EN, 0x0);

        /* Enable DAC */
        RADIO_MOD_BANK_REG(1, CH0_ADC_EN1, IDAC1_EN, 0x1);
        RADIO_MOD_BANK_REG(1, CH0_ADC_EN1, IDAC3_EN, 0x1);
        RADIO_MOD_BANK_REG(1, CH1_ADC_EN1, IDAC1_EN, 0x1);
        RADIO_MOD_BANK_REG(1, CH1_ADC_EN1, IDAC3_EN, 0x1);
        RADIO_MOD_BANK_REG(1, CH2_ADC_EN1, IDAC1_EN, 0x1);
        RADIO_MOD_BANK_REG(1, CH2_ADC_EN1, IDAC3_EN, 0x1);
        RADIO_MOD_BANK_REG(1, CH3_ADC_EN1, IDAC1_EN, 0x1);
        RADIO_MOD_BANK_REG(1, CH3_ADC_EN1, IDAC3_EN, 0x1);

        /* RXBB outmux selsection */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0,CH0_RX_T,OMUX_SEL,0x1);
        RADIO_MOD_BANK_REG(0,CH1_RX_T,OMUX_SEL,0x1);
        RADIO_MOD_BANK_REG(0,CH2_RX_T,OMUX_SEL,0x1);
        RADIO_MOD_BANK_REG(0,CH3_RX_T,OMUX_SEL,0x1);

        /* VCM Buf disable */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, CH0_ADC_EN0,BUFFER_VCMBUF_EN,0x0);
        RADIO_MOD_BANK_REG(1, CH1_ADC_EN0,BUFFER_VCMBUF_EN,0x0);
        RADIO_MOD_BANK_REG(1, CH2_ADC_EN0,BUFFER_VCMBUF_EN,0x0);
        RADIO_MOD_BANK_REG(1, CH3_ADC_EN0,BUFFER_VCMBUF_EN,0x0);

        /* 800MHz, signal bandwidth 40MHz & 50MHz */
        if ((cfg->adc_freq == 40) || (cfg->adc_freq == 50)){
                /* 800M CLK enable */
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_SDMADC_CLKSEL, 0x1);
                /* 800M ADC enable and reset */
                fmcw_radio_switch_bank(radio, 1);
                RADIO_WRITE_BANK_REG(1, CH0_ADC_EN0, 0xEF);
                RADIO_WRITE_BANK_REG(1, CH1_ADC_EN0, 0xEF);
                RADIO_WRITE_BANK_REG(1, CH2_ADC_EN0, 0xEF);
                RADIO_WRITE_BANK_REG(1, CH3_ADC_EN0, 0xEF);
        }

        /* ADC de-Reset */
        RADIO_MOD_BANK_REG(1, CH0_ADC_EN0, RST, 0x0);
        RADIO_MOD_BANK_REG(1, CH1_ADC_EN0, RST, 0x0);
        RADIO_MOD_BANK_REG(1, CH2_ADC_EN0, RST, 0x0);
        RADIO_MOD_BANK_REG(1, CH3_ADC_EN0, RST, 0x0);
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_rc_calibration(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, LDO25_SM, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO11_SM, LDO_EN, 0x1);

        //RC Calibration
        fmcw_radio_switch_bank(radio, 2);
        RADIO_MOD_BANK_REG(2, RCCAL, CLK_5M_EN, 0x1);
        RADIO_MOD_BANK_REG(2, RCCAL, VREFSEL, 0x4);
        RADIO_MOD_BANK_REG(2, RCCAL, PD, 0x1);

        /* Below Delay of 1ms is necessary for calibrated RC circuit to be settled. */
        MDELAY(1);

        RADIO_MOD_BANK_REG(2, RCCAL, START, 0x1);
        //Wait for 1ms
        MDELAY(1);

        /* The clock of RC calibration has to be turned off after calibration,
           otherwise it will cause unwanted spurs in BB FFT2D data. */
        RADIO_MOD_BANK_REG(2, RCCAL, CLK_5M_EN, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* single point frequency */
int32_t fmcw_radio_single_tone(fmcw_radio_t *radio, double freq, bool enable)
{
        uint32_t freq_reg;
        uint32_t freq_sel;

        freq_reg = DIV_RATIO(freq, FREQ_SYNTH_SD_RATE);
        fmcw_radio_do_pll_cal(radio, freq_reg);
        fmcw_radio_hp_auto_ch_off(radio,-1);
        if (fmcw_radio_is_pll_locked(radio) == false){
                return E_PLL_UNLOCK;
        }

        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SEL, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x0);
        RADIO_WRITE_BANK_REG(3, FMCW_MODE_SEL, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_SYNC, EN, 0x0);
        RADIO_MOD_BANK_REG(3, FMCW_SYNC, SEL, 0x3);
        RADIO_MOD_BANK_REG(3, FMCW_SYNC, EN, 0x1);
        /* get frame-interleaving frame type */
        uint8_t frame_type_id = RADIO_READ_BANK_REG(3, FMCW_FIL0);

        if (enable == true) {
                /* enable single_tone */
                freq_sel = freq_reg;
        } else {
                /* disable single_tone, restore init config */
                freq_sel = radio->start_freq;
        }

        fmcw_radio_switch_bank(radio, 5 + frame_type_id);
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_0,    REG_L(freq_sel));
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_1,    REG_M(freq_sel));
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_2,    REG_H(freq_sel));
        fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_3,    REG_INT(freq_sel));
        fmcw_radio_switch_bank(radio, 9);
        RADIO_WRITE_BANK_REG(9, LP_TST_FREQ_0, REG_L(freq_sel));
        RADIO_WRITE_BANK_REG(9, LP_TST_FREQ_1, REG_M(freq_sel));
        RADIO_WRITE_BANK_REG(9, LP_TST_FREQ_2, REG_H(freq_sel));
        RADIO_WRITE_BANK_REG(9, LP_TST_FREQ_3, REG_INT(freq_sel));
        fmcw_radio_switch_bank(radio, 3);
        RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x1);
        RADIO_MOD_BANK_REG(3, FMCW_START, START_SPI, 0x1);
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}


/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 * data          : HPF1_Hz   : 0x1
 *                 HPF1_Hz   : 0x2
 *                 HPF1_Hz   : 0x3
 *                 HPF1_Hz   : 0x4
 *                 HPF1_Hz   : 0x5
 *                 HPF1_Hz   : 0x6
 *                 HPF1_Hz   : 0x7
 */
void fmcw_radio_set_hpf1(fmcw_radio_t *radio, int32_t channel_index, int32_t filter_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        int ch;
        if (channel_index != -1){
                RADIO_MOD_BANK_CH_REG(0, channel_index, RX_TN2, HP1_SEL, filter_index);
        }
        else {
                for(ch = 0; ch < MAX_NUM_RX; ch++){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN2, HP1_SEL, filter_index);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_get_hpf1(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        char filter_index = RADIO_READ_BANK_CH_REG_FIELD(0, channel_index, RX_TN2, HP1_SEL);
        fmcw_radio_switch_bank(radio, old_bank);
        return filter_index;
}

/*
 * channel_index : 0 / 1 / 2 / 3 / -1
 * data          : HPF2_Hz   : 0x0
 *                 HPF2_Hz   : 0x1
 *                 HPF2_Hz   : 0x2
 *                 HPF2_Hz   : 0x3
 *                 HPF2_Hz   : 0x4
 *                 HPF2_Hz   : 0x5
 *                 HPF2_Hz   : 0x6
 */
void fmcw_radio_set_hpf2(fmcw_radio_t *radio, int32_t channel_index, int32_t filter_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        int ch;
        if (channel_index != -1){
                RADIO_MOD_BANK_CH_REG(0, channel_index, RX_TN2, HP2_SEL, filter_index);
        }
        else {
                for(ch = 0; ch < MAX_NUM_RX; ch++){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN2, HP2_SEL, filter_index);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

int32_t fmcw_radio_get_hpf2(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        char filter_index = RADIO_READ_BANK_CH_REG_FIELD(0, channel_index, RX_TN2, HP2_SEL);
        fmcw_radio_switch_bank(radio, old_bank);
        return filter_index;
}

/* enable fmcw auto on hp1 and hp2 */
void fmcw_radio_hp_auto_ch_on(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        if (channel_index == -1) {
                /* 0xF--on, ox0--off*/
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH0, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH1, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH2, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH3, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH0, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH1, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH2, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH3, EN, 0x1);
        } else if (channel_index == 0) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH0, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH0, EN, 0x1);
        } else if (channel_index == 1) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH1, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH1, EN, 0x1);
        } else if (channel_index == 2) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH2, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH2, EN, 0x1);
        } else if (channel_index == 3) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH3, EN, 0x1);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH3, EN, 0x1);
        } else {
                EMBARC_PRINTF("\n\n");
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

/* disable fmcw auto off hp1 and hp2 */
void fmcw_radio_hp_auto_ch_off(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        if (channel_index == -1) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH0, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH1, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH2, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH3, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH0, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH1, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH2, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH3, EN, 0x0);
        } else if (channel_index == 0) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH0, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH0, EN, 0x0);
        } else if (channel_index == 1) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH1, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH1, EN, 0x0);
        } else if (channel_index == 2) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH2, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH2, EN, 0x0);
        } else if (channel_index == 3) {
                RADIO_MOD_BANK_REG(3, AT_HPF1_CH3, EN, 0x0);
                RADIO_MOD_BANK_REG(3, AT_HPF2_CH3, EN, 0x0);
        } else {
                EMBARC_PRINTF("\n\n");
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

#if AUTO_TX == 1
/* enable fmcw auto on tx */
void fmcw_radio_tx_auto_ch_on(fmcw_radio_t *radio, int32_t channel_index, bool enable)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        int ch;
        if (channel_index == -1){
                for (ch = 0; ch < 4; ch++){
                        RADIO_MOD_BANK_CH_REG(3, ch, AT_TX, EN, enable);
                }
        }
        else{
                RADIO_MOD_BANK_CH_REG(3, channel_index, AT_TX, EN, enable);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

/*
 * set fmcw auto on tx mode
 * auto tx mode selection:
 * 0: idle start, down and idle state
 * 1: idle start and down state
 * 2: idle start and idle state
 * 3: idle start state
 * 4: down and idle state
 * 5: down state
 * 6: idle state
 */
void fmcw_radio_set_auto_tx_mode(fmcw_radio_t *radio, int32_t channel_index, int32_t mode_sel)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        int ch;
        if (channel_index == -1){
                for (ch = 0; ch < 4; ch++){
                        RADIO_MOD_BANK_CH_REG(3, ch, AT_TX, SEL, mode_sel);
                }
        }
        else{
                RADIO_MOD_BANK_CH_REG(3, channel_index, AT_TX, SEL, mode_sel);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}
#endif

/* SDM reset */
void fmcw_radio_sdm_reset(fmcw_radio_t *radio)
{
        fmcw_radio_adc_on(radio);
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        uint8_t ch; /* channel index*/
        for(ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, RST, 1);
        }
        /* after 4 channel reset asserted, then deasserted*/
        for(ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, RST, 0);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

/* enable/disable fmcw agc mode */
void fmcw_radio_agc_enable(fmcw_radio_t *radio, bool enable)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        int val = (enable == true)?1:0;
        RADIO_MOD_BANK_REG(5,FMCW_AGC_EN_1,AGC_EN_1,val);   //agc enable
        fmcw_radio_switch_bank(radio, 0);
        for (int ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_VGA, val);     //RF BB VGA2 enable
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_EN, val);      //RF BB VGA1 enable
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, SAT_EN, val);      //TIA SAT enable
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_agc_setup(fmcw_radio_t *radio)
{
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        //  code start
        for (int ch = 0; ch < MAX_NUM_RX; ch++) {
                //TIA_SAT_VREF config
                if (cfg->agc_tia_thres >= 0.5){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, TIA_SAT_VREF_SEL,0x3);  //TIA vREf 1v
                }
                else if(cfg->agc_tia_thres > 0.45){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, TIA_SAT_VREF_SEL,0x2);  //TIA vREf 0.95v
                }
                else if(cfg->agc_tia_thres > 0.43){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, TIA_SAT_VREF_SEL,0x1);  //TIA vREf 0.9v
                }
                else{
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, TIA_SAT_VREF_SEL,0x0);  //TIA vREf 0.85v
                 //VGA_SAT_VREF config
                }
                if (cfg->agc_vga1_thres >= 1.0){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_VTHSEL,0x3);  //VGA vREf 1.7v
                }
                else if (cfg->agc_vga1_thres >= 0.95){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_VTHSEL,0x2);  //VGA vREf 1.65v
                }
                else if (cfg->agc_vga1_thres >= 0.8){
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_VTHSEL,0x1);  //VGA vREf 1.55v
                }
                else{
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_VTHSEL,0x0);  //VGA vREf 1.5v
                }
        }
        //  code end
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_vam_status_save(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        vam_status[0] = RADIO_READ_BANK_REG(5, FMCW_TX0_CTRL_1_2);
        vam_status[1] = RADIO_READ_BANK_REG(5, FMCW_TX1_CTRL_1_2);
        vam_status[2] = RADIO_READ_BANK_REG(5, FMCW_TX2_CTRL_1_2);
        vam_status[3] = RADIO_READ_BANK_REG(5, FMCW_TX3_CTRL_1_2);
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_txphase_status(fmcw_radio_t *radio, bool save)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        int ch;
        if (save) {
                for (ch=0; ch<MAX_NUM_TX; ch++) {
                       txphase_status[2*ch]     = RADIO_READ_BANK_CH_REG(1, ch, TX_TN0);
                       txphase_status[2*ch + 1] = RADIO_READ_BANK_CH_REG(1, ch, TX_TN1);
                }
        } else {
                for (ch=0; ch<MAX_NUM_TX; ch++) {
                       RADIO_WRITE_BANK_CH_REG(1, ch, TX_TN0, txphase_status[2*ch]);
                       RADIO_WRITE_BANK_CH_REG(1, ch, TX_TN1, txphase_status[2*ch+1]);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_vam_disable(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        for (uint8_t ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_FWCW_TX_REG(5, ch, CTRL_1_2, VAM_EN, 0x0);  /* disable varray mode to write spi registers */
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_vam_status_restore(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        RADIO_WRITE_BANK_REG(5, FMCW_TX0_CTRL_1_2, vam_status[0]);
        RADIO_WRITE_BANK_REG(5, FMCW_TX1_CTRL_1_2, vam_status[1]);
        RADIO_WRITE_BANK_REG(5, FMCW_TX2_CTRL_1_2, vam_status[2]);
        RADIO_WRITE_BANK_REG(5, FMCW_TX3_CTRL_1_2, vam_status[3]);
        fmcw_radio_switch_bank(radio, old_bank);
}

/* For protection of switching between some special modes without power off the chirp */
void fmcw_radio_special_mods_off(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        //turn off VAM mode
        for (int ch = 0; ch < MAX_NUM_TX; ch++) {
                RADIO_WRITE_BANK_FWCW_TX_REG(5, ch, CTRL_1_1, 0x0);
                RADIO_MOD_BANK_FWCW_TX_REG(5, ch, CTRL_1_2, VAM_EN, 0x0);
                RADIO_MOD_BANK_FWCW_TX_REG(5, ch, CTRL_1_2, VAM_P,  0x0);
        }
        //turn off ps mode
        fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        RADIO_MOD_BANK_REG(5, FMCW_PS_EN_1, PS_EN_1, 0x0);
        //turn off AGC mode
        RADIO_MOD_BANK_REG(5, FMCW_AGC_EN_1, AGC_EN_1,0x0);

        fmcw_radio_switch_bank(radio, 0);
        for (int ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_VGA, 0x0);     //RF BB VGA2 enable
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_EN, 0x0);      //RF BB VGA1 enable
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, SAT_EN, 0x0);      //TIA SAT enable
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

uint32_t fmcw_radio_compute_lock_freq(fmcw_radio_t *radio)
{
        uint32_t min_freq = 0;
        uint32_t max_freq = 0;
        uint32_t start_freq;
        uint32_t stop_freq;
        uint32_t max_bandwidth;
        for (uint8_t i = 0; i < NUM_FRAME_TYPE; i++) {
                sensor_config_t *cfg = sensor_config_get_config(i);
                start_freq = DIV_RATIO(cfg->fmcw_startfreq, FREQ_SYNTH_SD_RATE);
                stop_freq  = DIV_RATIO(cfg->fmcw_startfreq + cfg->fmcw_bandwidth * 1e-3, FREQ_SYNTH_SD_RATE);
                if(i == 0) {
                        min_freq = start_freq;
                        max_freq = stop_freq;
                } else {
                        if (min_freq > start_freq){
                                min_freq = start_freq;
                        }
                        if (max_freq < stop_freq){
                                max_freq = stop_freq;
                        }
                }
        }
        max_bandwidth = max_freq - min_freq;
        uint32_t lock_freq = min_freq + max_bandwidth / 2;
        return lock_freq;
}

/*Bug fix for anti velamb glitch*/
void fmcw_radio_cmd_cfg(fmcw_radio_t *radio, bool enable)
{
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;

        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        //switch to bank 5 without return value
        fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        //CMD period
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_PRD_1, cfg->nvarray + 1);

        /* CMD1 config */
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_0, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_1, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_2, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_3, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_NUM_1_0    , 0x0 );

        /* CMD2 config */
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_0, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_1, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_2, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_3, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_NUM_1_1    , 0x0);

        /* CMD3 config */
        if(enable){
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_0, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >>  0) & 0xff);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_1, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >>  8) & 0xff);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_2, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >> 16) & 0xff);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_3, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >> 24) & 0xff);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_NUM_1_2, MAX_NUM_TX);
        }else{
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_0, 0);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_1, 0);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_2, 0);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_3, 0);
                RADIO_WRITE_BANK_REG(5, FMCW_CMD_NUM_1_2, 0x0);
        }

        //switch to bank 4 without return value
        fmcw_radio_switch_bank(radio, 4);

        //CMD Group1, switch to bank 1, for the extra chirp
        for(int index = 0; index < MAX_NUM_TX; index++ ){
                fmcw_radio_reg_write(radio, R4_CPU_ADDR1 + 2 * index , 0x00);
                fmcw_radio_reg_write(radio, R4_CPU_DATA1 + 2 * index , 0x01);
        }

        //parse tx_groups, concat head and tail
        uint32_t tx_groups_extend[MAX_NUM_TX] ={0};
        for( int ch = 0; ch < MAX_NUM_TX; ch++){
                tx_groups_extend[ch] = ( ( cfg->tx_groups[ch] & 0xF ) << ( 4 * cfg->nvarray ) ) + cfg->tx_groups[ch] ;
        }

        //CMD Group2 and CMD group 3
        for(int prd = 0; prd < (cfg->nvarray); prd++ ){
                uint8_t grp[MAX_NUM_TX] = {0};
                for( int i = 0 ; i < MAX_NUM_TX ; i++ ){
                      grp[i] = ( tx_groups_extend[i] >> ( 4 * prd ) ) & 0xFF;
                }

                for( int ch = 0; ch < MAX_NUM_TX; ch++){
                        switch ( grp[ch] ){
                                case 0x11:
                                case 0x10:
                                        //on current tx channel
                                        fmcw_radio_reg_write(radio, R4_CPU_ADDR5 + 2*ch + 2 * prd * 4 , R1_CH0_TX_EN0 + ch * 9);  //address
                                        fmcw_radio_reg_write(radio, R4_CPU_DATA5 + 2*ch + 2 * prd * 4, 0x0f                         );  //data
                                        break;
                                case 0x00:
                                case 0x01:
                                        //off current tx channel
                                        fmcw_radio_reg_write(radio, R4_CPU_ADDR5 + 2*ch + 2 * prd * 4 , R1_CH0_TX_EN0 + ch * 9);  //address
                                        fmcw_radio_reg_write(radio, R4_CPU_DATA5 + 2*ch + 2 * prd * 4 , 0x03                         );  //data
                                        break;
                                default:
                                        EMBARC_PRINTF("\n\n");
                                        break;
                        }
                }

        }
        //code end
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_auxadc_trim(fmcw_radio_t *radio)
{
        uint16_t trim_data = baseband_read_reg(NULL,OTP_TRIM_AUX_ADC);
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        if( trim_data != 0 ){
                RADIO_MOD_BANK_REG(1, DTSMD1_SEL, DTSDM_REFSEL, REG_M(trim_data));
                RADIO_MOD_BANK_REG(1, DTSMD2_SEL, DTSDM_REFSEL, REG_L(trim_data));
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

uint32_t fmcw_radio_rfbist_trim(fmcw_radio_t *radio)
{
        uint32_t trim_data = baseband_read_reg(NULL,OTP_TRIM_RF_BIST);
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        fmcw_radio_switch_bank(radio, old_bank);
        return trim_data;
}

void fmcw_radio_temp_sensor_trim(fmcw_radio_t *radio)
{
    /* read 32-bit temp sensor trim data */
    uint32_t ts_low_trim_data = baseband_read_reg(NULL,OTP_TRIM_TEMP_SENSOR_LOW);
    uint32_t ts_high_trim_data = baseband_read_reg(NULL,OTP_TRIM_TEMP_SENSOR_HIGH);
    /* trim data bit conversion */
    uint8_t ts_high_temp = REG_INT(ts_high_trim_data);
    uint8_t ts_low_temp = REG_INT(ts_low_trim_data);
    uint32_t ts_high_dout = ts_high_trim_data & 0x03ffff;
    uint32_t ts_low_dout = ts_low_trim_data & 0x03ffff;
    /* store in ts_coefficient array */
    float ts_k = 0, ts_d = 0, ts_t = 0;
    if (ts_low_temp) {
        /* two point calibration */
        ts_k = (float)(ts_high_dout - ts_low_dout)/(float)(ts_high_temp - ts_low_temp);
        ts_d = (float)ts_high_dout;
        ts_t = (float)(ts_high_temp - 55);
        ts_coefficient[0] = ts_k;
        ts_coefficient[1] = ts_d;
        ts_coefficient[2] = ts_t;
    } else if (ts_high_temp) {
        /* one point calibration */
        ts_d = (float)ts_high_dout;
        ts_t = (float)(ts_high_temp - 55);
        ts_coefficient[0] = TS_K;
        ts_coefficient[1] = ts_d;
        ts_coefficient[2] = ts_t;
    } else {
        /* no calibration */
        ts_coefficient[0] = TS_K;
        ts_coefficient[1] = TS_D;
        ts_coefficient[2] = TS_T;
    }
}

uint8_t fmcw_radio_part_number(fmcw_radio_t *radio)
{
        uint8_t part_no = REG_L8(baseband_read_reg(NULL,OTP_PART_NO));
        return part_no;
}

int32_t fmcw_radio_lvds_on(fmcw_radio_t *radio, bool enable)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);
        int ch;
        RADIO_MOD_BANK_REG(2, LVDS_LDO25, LDO25_LVDS_LDO_EN, enable);
        for (ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_CH_REG(2, ch, LVDS, EN, enable);
                RADIO_MOD_BANK_CH_REG(2, ch, LVDS, PRE_EN, enable);
        }
        RADIO_MOD_BANK_REG(2, LVDS_CLK, EN, enable);
        RADIO_MOD_BANK_REG(2, LVDS_CLK, PRE_EN, enable);
        RADIO_MOD_BANK_REG(2, LVDS_FRAME, EN, enable);
        RADIO_MOD_BANK_REG(2, LVDS_FRAME, PRE_EN, enable);
        RADIO_MOD_BANK_REG(2, ADC_FILTER0, RSTN, enable);
        RADIO_MOD_BANK_REG(2, ADC_FILTER0, LVDS_O_EN, enable);
        RADIO_MOD_BANK_REG(2, ADC_FILTER0, CMOS_O_EN, enable);
        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

/*
 * inputmux selection : MainBG VPTAT(default) / TestMUXN / TestMUXP / TPANA1
 *      #define AUXADC1_MainBG_VPTAT     0
 *      #define AUXADC1_TestMUXN         1
 *      #define AUXADC1_TestMUXP         2
 *      #define AUXADC1_TPANA1           3
 *      #define AUXADC1_T_POINT_MUX   4
 */
float fmcw_radio_auxadc1_voltage(fmcw_radio_t *radio, int32_t muxin_sel)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Enable 800M ADC CLK */
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);

        /* Enable safety monitor LDO */
        RADIO_MOD_BANK_REG(0, LDO25_SM, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO11_SM, LDO_EN, 0x1);

#if defined(FUNC_SAFETY) && (SAFETY_FEATURE_SM2 == 1)
        /* Clear IRQ of LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
        /* Disable AVDD33 Monitor */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_RDAC_EN, 0x0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_EN, 0x0);
#endif
        /* Enable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x1);

        /* AUXADC1 on and reset */
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_BUF_BP, 0x0);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_BUF_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_BI_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_OP1_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_OP2_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_REFGEN_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_VCMGEN_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_RST, 0x1);

        /* select AUXADC1 InputMUX */
        switch (muxin_sel) {
        case 0:
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, T_CBC2, VPTAT_T_EN, 0x1);
                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, DTSDM_MUXIN_SEL, 0x0);
                break;
        case 1:
                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_MUXN_EN, 0x1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, DTSDM_MUXIN_SEL, 0x1);
                break;
        case 2:
                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_MUXP_EN, 0x1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, DTSDM_MUXIN_SEL, 0x2);
                break;
        case 3:
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, TPANA1, EN, 0x1);
                RADIO_MOD_BANK_REG(0, TPANA1, T_MUX_1_EN, 0x0);
                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, DTSDM_MUXIN_SEL, 0x3);
                break;
        case 4:
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, TPANA1, EN, 0x0);
                RADIO_MOD_BANK_REG(0, TPANA1, T_MUX_1_EN, 0x1);
                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, DTSDM_MUXIN_SEL, 0x3);
                break;
        default:
                EMBARC_PRINTF("\n\n");
                break;
        }

        /* AUXADC1 de-reset */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSDM1_EN, DTSDM_RST, 0x0);

        /* AUXADC1 Filter de-reset */
        fmcw_radio_switch_bank(radio, 2);
        RADIO_WRITE_BANK_REG(2, DC_FILTER1_RST_EN, 0x01);
        MDELAY(2);

        /* Disable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        /* Disable AUXADC1 */
        RADIO_WRITE_BANK_REG(1, DTSDM1_EN, 0x0);

        /* read back AUXADC1 Filter Output Digital Bits */
        uint32_t doutL, doutM, doutH;
        float dout;
        doutL = RADIO_READ_BANK_REG(1, DTSDM1_DAT0);
        doutM = RADIO_READ_BANK_REG(1, DTSDM1_DAT1);
        doutH = RADIO_READ_BANK_REG(1, DTSDM1_DAT2);
        dout = doutL + (doutM << 8) + (doutH << 16);

        /* return voltage measurement, formula refer to 85C */
        float auxadc1_voltage;
        auxadc1_voltage = (dout / (1<<17) -1) * 3.3 + 1.67;

#if defined(FUNC_SAFETY) && (SAFETY_FEATURE_SM2 == 1)
        fmcw_radio_sm_avdd33_monitor_IRQ(radio, false);
#endif
        fmcw_radio_switch_bank(radio, old_bank);
        return auxadc1_voltage;
}

void fmcw_radio_clk_out_for_cascade(void)
{
#ifdef CHIP_CASCADE
        uint8_t old_bank = fmcw_radio_switch_bank(NULL, 0);

        if (chip_cascade_status() == CHIP_CASCADE_MASTER) {
                RADIO_MOD_BANK_REG(0, RP_EN, CLK_400M_FMCW_EN, 0x1);
                MDELAY(200);
                fmcw_radio_switch_bank(NULL, 2);
#if FMCW_SDM_FREQ == 400
                RADIO_MOD_BANK_REG(2, DIV_CLK, FLTR_CLK_DIV_RATIO, 0x4);
#elif FMCW_SDM_FREQ == 360
                RADIO_MOD_BANK_REG(2, DIV_CLK, FLTR_CLK_DIV_RATIO, 0x5);
#endif
                RADIO_MOD_BANK_REG(2, DIV_CLK, FLTR_CLK_O_ENABLE, 0x1);
        } else {
                RADIO_MOD_BANK_REG(0, RP_EN, CLK_400M_FMCW_EN, 0x0);
        }

        fmcw_radio_switch_bank(NULL, old_bank);
#endif
}

/*
 * inputmux selection : TS VPTAT(default) / TS VBG / TestMUXN / TPANA2
 *      #define AUXADC2_TS_VPTAT         0
 *      #define AUXADC2_TS_VBG           1
 *      #define AUXADC2_TestMUXN         2
 *      #define AUXADC2_TPANA2           3
 *      #define AUXADC2_T_POINT_MUX   4
 */
float fmcw_radio_auxadc2_voltage(fmcw_radio_t *radio, int32_t muxin_sel)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Enable 800M ADC CLK */
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);
        /* Enable safety monitor LDO */
        RADIO_MOD_BANK_REG(0, LDO25_SM, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO11_SM, LDO_EN, 0x1);

#if defined(FUNC_SAFETY) && (SAFETY_FEATURE_SM2 == 1)
        /* Clear IRQ of LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
        /* Disable AVDD33 Monitor */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_RDAC_EN, 0x0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_EN, 0x0);
#endif
        /* Enable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x1);

        /* AUXADC2 on and reset */
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_BUF_BP, 0x0);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_BUF_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_BI_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_OP1_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_OP2_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_REFGEN_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_VCMGEN_EN, 0x1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_RST, 0x1);

        /* select AUXADC2 InputMUX */
        switch (muxin_sel) {
        case 0:
                fmcw_radio_switch_bank(radio, 1);
                RADIO_WRITE_BANK_REG(1, DTSMD2_MUXIN_SEL, 0x90);
                break;
        case 1:
                fmcw_radio_switch_bank(radio, 1);
                RADIO_WRITE_BANK_REG(1, DTSMD2_MUXIN_SEL, 0x91);
                break;
        case 2:
                fmcw_radio_switch_bank(radio, 1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_MUXN_EN, 0x1);
                RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_MUXP_EN, 0x1);
                RADIO_WRITE_BANK_REG(1, DTSMD2_MUXIN_SEL, 0x92);
                break;
        case 3:
                fmcw_radio_switch_bank(radio, 1);
                RADIO_WRITE_BANK_REG(1, DTSMD2_MUXIN_SEL, 0x93);
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, TPANA2, TPANA2_EN, 0x1);
                RADIO_MOD_BANK_REG(0, TPANA2, T_MUX_2_EN, 0x0);
                break;
        case 4:
                fmcw_radio_switch_bank(radio, 1);
                RADIO_WRITE_BANK_REG(1, DTSMD2_MUXIN_SEL, 0x93);
                fmcw_radio_switch_bank(radio, 0);
                RADIO_MOD_BANK_REG(0, TPANA2, TPANA2_EN, 0x0);
                RADIO_MOD_BANK_REG(0, TPANA2, T_MUX_2_EN, 0x1);
                break;
        default:
                EMBARC_PRINTF("\n\n");
                break;
        }

        /* AUXADC2 de-reset */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSDM2_EN, DTSDM_RST, 0x0);

        /* AUXADC2 Filter de-reset */
        fmcw_radio_switch_bank(radio, 2);
        RADIO_WRITE_BANK_REG(2, DC_FILTER2_RST_EN, 0x01);
        MDELAY(2);
        /* Disable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        /* Disable AUXADC2 */
        RADIO_WRITE_BANK_REG(1, DTSDM2_EN, 0x0);

        /* read back AUXADC2 Filter Output Digital Bits */
        uint32_t doutL, doutM, doutH;
        float dout;
        doutL = RADIO_READ_BANK_REG(1, DTSDM2_DAT0);
        doutM = RADIO_READ_BANK_REG(1, DTSDM2_DAT1);
        doutH = RADIO_READ_BANK_REG(1, DTSDM2_DAT2);
        dout = doutL + ( doutM << 8 ) + ( doutH << 16 );

        /* return voltage measurement, formula refer to 85C */
        float auxadc2_voltage;
        auxadc2_voltage = (dout / (1<<17) -1) * 3.3 + 1.67;

#if defined(FUNC_SAFETY) && (SAFETY_FEATURE_SM2 == 1)
        fmcw_radio_sm_avdd33_monitor_IRQ(radio, false);
#endif
        fmcw_radio_switch_bank(radio, old_bank);
        return auxadc2_voltage;
}

#ifdef FUNC_SAFETY
/* This function is used for getting IRQ value of Safety Monitor1: LDO Monitor */
uint8_t fmcw_radio_sm_ldo_monitor_IRQ(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        uint8_t IRQ_value;

        /* Read IRQ */
        IRQ_value = RADIO_READ_BANK_REG_FIELD(10,ITF_IRQ_1,ITF_IRQ_SUPPLY_LDO);

        /* resume ldo monitor */
        fmcw_radio_sm_ldo_monitor_IRQ_resume(radio);
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used for fault injection of Safety Monitor1: LDO Monitor */
uint8_t fmcw_radio_sm_ldo_monitor_fault_injection_IRQ(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);

        uint8_t old_ldo;
        uint8_t IRQ_value;

        /* set adc 1.1V voltage selection to highest vaule */
        old_ldo = RADIO_READ_BANK_REG_FIELD(2, BIST_LDO, LDO11_BIST_EN);
        RADIO_MOD_BANK_REG(2, BIST_LDO, LDO11_BIST_EN, 0x0);

        /* enable AUXADC1 */
        fmcw_radio_auxadc1_voltage(radio,4);

        /* enable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x1);

        /* set threshold of LDO Monitor */
        fmcw_radio_sm_ldo_monitor_threshold(radio);

        /* power on self test */
        EMBARC_PRINTF("reached sm ldo monitor fault injection\n");
        /* enable LDO Monitor of LDO11_ADC */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_WRITE_BANK_REG(10, LDO_MON_EN_3, 0x40);
        /* set threshold of LDO Monitor timing */
        RADIO_WRITE_BANK_REG(10, LDO_MON_TIME_CNT_1, 0x3F);
        RADIO_WRITE_BANK_REG(10, LDO_MON_CTU_SIZE, 0x18);
        RADIO_WRITE_BANK_REG(10, LDO_MON_RSTN, 0x1);

        /* clear IRQ of ldo monitor */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_CLEAR, 0x1);
        UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_CLEAR, 0x0);

        /* start ldo monitor */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_T_START, 0x1);
        UDELAY(300);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_T_START, 0x0);
        /* ldo monitor needs delay to get accurate monitor result */
        MDELAY(4);

        /* Read IRQ */
        IRQ_value = RADIO_READ_BANK_REG_FIELD(10,ITF_IRQ_1,ITF_IRQ_SUPPLY_LDO);

        /* resume ldo status */
        fmcw_radio_switch_bank(radio, 2);
        RADIO_MOD_BANK_REG(2, BIST_LDO, LDO11_BIST_EN, old_ldo);

        /* disable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        /* disable LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_WRITE_BANK_REG(10, LDO_MON_EN_3, 0x00);
        RADIO_WRITE_BANK_REG(10, LDO_MON_RSTN, 0x0);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_CLEAR, 0x1);
        UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_CLEAR, 0x0);
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used for getting IRQ value of Safety Monitor2: AVDD33 Monitor
 * 111 AVDD33 > +10%
 * 101 +7.5% < AVDD33 < +10%
 * 000 -7.5% < AVDD33 < +7.5%
 * 110 -10% < AVDD33 < -7.5%
 * 100 AVDD33 < -10%
 * fault_injection = true: enter power on self test
 * fault_injection = false: leave power on self test/normal monitoring
 */
uint8_t fmcw_radio_sm_avdd33_monitor_IRQ(fmcw_radio_t *radio, bool fault_injection)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        uint8_t IRQ_value_L,IRQ_value_H,IRQ_value;
        /* get avdd33 monitor threahold */
        fmcw_radio_sm_avdd33_monitor_threshold(radio, fault_injection);
        /* get avdd33 monitor setting */
        fmcw_radio_sm_avdd33_monitor_setting(radio);

        /* Read IRQ */
        IRQ_value_H = RADIO_READ_BANK_REG_FIELD(10,ITF_IRQ_1,ITF_IRQ_SUPPLY_CBC33);
        fmcw_radio_switch_bank(radio, 0);
        IRQ_value_L = RADIO_READ_BANK_REG(0,MS)& 0x03;
        IRQ_value = (IRQ_value_H << 2) + IRQ_value_L;

        /* restore avdd33 threahold and clear IRQ status for monitoring */
        if (fault_injection) {
                fmcw_radio_sm_avdd33_monitor_threshold(radio, false);
                fmcw_radio_switch_bank(radio, 10);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x0);
        }

        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used for getting IRQ value of Safety Monitor3: DV11 Monitor
 * fault_injection = true: enter power on self test
 * fault_injection = false: leave power on self test/normal monitoring
 */
uint8_t fmcw_radio_sm_dvdd11_monitor_IRQ(fmcw_radio_t *radio, bool fault_injection)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        uint8_t IRQ_value;

        /* get dvdd11 monitor threahold */
        fmcw_radio_sm_dvdd11_monitor_threshold(radio, fault_injection);
        /* get dvdd11 monitor setting */
        fmcw_radio_sm_dvdd11_monitor_setting(radio);

        /* Read IRQ */
        IRQ_value = RADIO_READ_BANK_REG_FIELD(10,ITF_IRQ_1,ITF_IRQ_SUPPLY_DV11);

        /* restore dvdd11 threahold and clear IRQ status for monitoring */
        if (fault_injection) {
                fmcw_radio_sm_dvdd11_monitor_threshold(radio, false);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_DV11_CLEAR, 0x1);
                // UDELAY(100);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_DV11_CLEAR, 0x0);
        }

        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used for getting IRQ value of Safety Monitor4: Bandgap Voltage Monitor
 * fault_injection = true: enter power on self test
 * fault_injection = false: leave power on self test/normal monitoring
 */
uint8_t fmcw_radio_sm_bg_monitor_IRQ(fmcw_radio_t *radio, bool fault_injection)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        uint8_t IRQ_value;

        /* get bg monitor threahold */
        fmcw_radio_sm_bg_monitor_threshold(radio, fault_injection);
        /* get bg monitor setting */
        fmcw_radio_sm_bg_monitor_setting(radio);

        /* Read IRQ */
        IRQ_value = RADIO_READ_BANK_REG_FIELD(10,ITF_IRQ_1,ITF_IRQ_VBG);

        /* restore dvdd11 threahold and clear IRQ status for monitoring */
        if (fault_injection) {
                fmcw_radio_sm_bg_monitor_threshold(radio, false);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_VBG_CLEAR, 0x1);
                // UDELAY(100);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_VBG_CLEAR, 0x0);
        }

        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/*
 * This function is used for getting IRQ value of Safety Monitor fault injection: RF Power Detector
 * freq            : lock freq
 * power_th        : rf power lower threahold (min 5dBm, max 20dBm)
 * channel_index   : 0/ 1/ 2/ 3/ -1 (all channels)
 * fault injection : true
 */
uint8_t fmcw_radio_sm_rfpower_detector_fault_injection_IRQ(fmcw_radio_t *radio, double power_th)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint32_t channel_index;
        double freq;

        uint8_t IRQ_value;
        /* set rfpower detector fault injection threshold */
        fmcw_radio_sm_rfpower_detector_fault_injection_threshold(radio);
        /* get IRQ */
        IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(radio);

        /* get parameters rf power detector threshold needs */
        channel_index = fmcw_radio_check_txon_channel(radio);
        freq = cfg->fmcw_startfreq;
        EMBARC_PRINTF("CH%d on, freq = %.2f, threshold = %.2f\n",channel_index,freq,power_th);
        /* set rfpower detector threshold for monitoring*/
        fmcw_radio_sm_rfpower_detector_threshold(radio, freq, power_th, channel_index);

        /* clear IRQ status for monitoring */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_CLEAR, 0x1);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_CLEAR, 0x0);

        /* resume all settings */
        fmcw_radio_pdt_reg_resume(radio, channel_index);
        fmcw_radio_sm_rf_power_detector_IRQ_resume(radio);
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/*
 * This function is used for getting IRQ value of Safety Monitor: RF Power Detector
 * freq            : lock freq
 * power_th        : rf power lower threahold (min 5dBm)
 * channel_index   : 0/ 1/ 2/ 3/ -1 (all channels)
 * fault injection : false
 */
uint8_t fmcw_radio_sm_rfpower_detector_IRQ(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t IRQ_value = 0;
        uint32_t ch;
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;

        /* check the current enabled tx channel */
        ch = fmcw_radio_check_txon_channel(radio);
        //EMBARC_PRINTF("CH%d on\n",ch);
        if (ch == 4) {
                /* no channel is on, IRQ = 1 */
                // IRQ_value = 1;
                fmcw_radio_switch_bank(radio, old_bank);
                return IRQ_value;
        } else {
                for (ch = 0; ch < MAX_NUM_TX; ch++) {
                        if (cfg->tx_groups[ch] > 0) {
                                /* get pa on dout value */
                                fmcw_radio_pdt_reg(radio, 2, ch);
                                /* set voltage measurement to auxadc2 internal test point */
                                // fmcw_radio_auxadc2_voltage(radio, 4);//unneeded,in fmcw_radio_sm_rfpower_detector_setting has the same operation
                                /* get IRQ value */
                                fmcw_radio_sm_rfpower_detector_setting(radio, ch);
                                // EMBARC_PRINTF("ch =%d\n",ch);
                                fmcw_radio_switch_bank(radio, 10);
                                /* Read IRQ */
                                IRQ_value = IRQ_value || RADIO_READ_BANK_REG_FIELD(10, ITF_IRQ_2, ITF_IRQ_RFPOWER);
                        }
                }
                /* resume all settings */
                fmcw_radio_sm_rf_power_detector_IRQ_resume(radio);
                fmcw_radio_pdt_reg_resume(radio, ch);
#if (SAFETY_FEATURE_SM2 == 1)
                fmcw_radio_sm_avdd33_monitor_IRQ(radio, false);
#endif
                fmcw_radio_switch_bank(radio, old_bank);
                return IRQ_value;
        }
}

/* This function is used for Safety Monitor: IF Loopback */
uint8_t fmcw_radio_sm_if_loopback_IRQ(fmcw_radio_t *radio, bool fault_injection)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t ch, IRQ_value = 0;
        float vga_gain_tia250[4], vga_gain_tia500[4];
        safety_monitor_mode = true;
        baseband_t *bb = baseband_get_bb(0);
        baseband_hw_t *bb_hw = &bb->bb_hw;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        uint8_t agc_sta = RADIO_READ_BANK_REG(5,FMCW_AGC_EN_1) & 0x01;

        if(agc_sta)
        {
                RADIO_MOD_BANK_REG(5,FMCW_AGC_EN_1,AGC_EN_1,0);
        }
        /* enable 400M ADC CLK */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, PLL_CLK_O, RP_CLK_800M_ADC_EN, 0x1);

        /* Fault Injection */
        if (fault_injection) {
                EMBARC_PRINTF("reached sm11 if loopback fault injection\n");
                /* set rx gain*/
                for (ch = 0; ch < MAX_NUM_RX; ch++) {
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, 0xf);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, 0x1);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, 0x1);
                }
        }
        else
        {
                /* set rx gain 250 1 1 */
                for (ch = 0; ch < MAX_NUM_RX; ch++) {
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, 0x1);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, 0x1);
                        RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, 0x1);
                }
        }
        // MDELAY(2);
        // EMBARC_PRINTF("time_1 = %f\n",fusa_get_current_time_ms());
        baseband_dac_playback(bb_hw, true, 0, 3, false, vga_gain_tia250);
        /* set rx gain 500 3 3*/
        for (ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, 0x2);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, 0x3);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, 0x3);
        }
        // MDELAY(2);
        // EMBARC_PRINTF("time_2 = %f\n",fusa_get_current_time_ms());
        baseband_dac_playback(bb_hw, true, 0, 3, false, vga_gain_tia500);
        // EMBARC_PRINTF("time_3 = %f\n",fusa_get_current_time_ms());
        for (ch = 0; ch < MAX_NUM_RX; ch++) {
                if ((vga_gain_tia500[ch] - vga_gain_tia250[ch] >= 16) && ((vga_gain_tia500[ch] - vga_gain_tia250[ch]) <= 20)) {
                        continue;
                } else {
                        IRQ_value = 1;
                        break;
                }
        }

        safety_monitor_mode = false;

        /* restore rx gain setting */
        for (ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, cfg->rf_tia_gain);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, cfg->rf_vga2_gain);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, cfg->rf_vga1_gain);
        }
        /* restore agc setting */
        if(agc_sta)
        {
                fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
                RADIO_MOD_BANK_REG(5,FMCW_AGC_EN_1,AGC_EN_1,1);
        }
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used for Safety Monitor: RF Loopback */
uint8_t fmcw_radio_sm_rf_loopback_IRQ(fmcw_radio_t *radio, bool fault_injection)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t ch, IRQ_value=0;
        float rf_gain[4], rf_gain_off[4];
        safety_monitor_mode = true;
        baseband_t *bb = baseband_get_bb(0);
        baseband_hw_t *bb_hw = &bb->bb_hw;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        uint8_t agc_sta = RADIO_READ_BANK_REG(5,FMCW_AGC_EN_1) & 0x01;

        if(agc_sta)
        {
                // RADIO_MOD_BANK_REG(5,FMCW_AGC_EN_1,AGC_EN_1,0);
                fmcw_radio_agc_enable(radio,false);
        }
        fmcw_radio_switch_bank(radio, 0);
        uint8_t old_rxlo_ldo = RADIO_READ_BANK_REG_FIELD(0, LO_LDO3, LDO11_RXLO_VO_SEL);
        // MDELAY(1);
        /* set rx gain */
        for (ch = 0; ch < 4; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, LNA_GC, 0xf);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, 0x1);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, 0x1);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, 0x1);
        }

        /* get rx bist value */
        baseband_dac_playback(bb_hw, false, 0, 3, false, rf_gain);
        rf_loopback_mode = true;
        // MDELAY(2);
        /* disable rx lobuff and lna bias */
        for (ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, RLB_BI_EN, false);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, LNA2_BI_EN, false);
        }
        /* set rxlo ldo selection to 0 */
        RADIO_MOD_BANK_REG(0, LO_LDO3, LDO11_RXLO_VO_SEL, 0x0);
        /* Fault Injection */
        if (fault_injection) {
                EMBARC_PRINTF("reached sm12 rf loopback fault injection\n");
                RADIO_MOD_BANK_REG(0, RX_LDO0, LDO11_RFN_EN, 0x0);
                RADIO_MOD_BANK_REG(0, RX_LDO1, LDO11_RFS_EN, 0x0);
                baseband_dac_playback(bb_hw, false, 0, 3, false, rf_gain_off);
                for (ch = 0; ch < MAX_NUM_RX; ch++) {
                        if (rf_gain[ch] - rf_gain_off[ch] > 30) {
                                IRQ_value = 1;
                                continue;
                        } else {
                                IRQ_value = 0;
                                break;
                        }
                }
        } else {
                baseband_dac_playback(bb_hw, false, 0, 3, false, rf_gain_off);
                /* Normal Operation */
                for (ch = 0; ch < MAX_NUM_RX; ch++) {
                        if ((rf_gain[ch] - rf_gain_off[ch])>= 15) {
                                continue;
                        } else {
                                IRQ_value = 1;
                                break;
                        }
                }
        }

        /* Fault Injection Release*/
        if (fault_injection) {
                RADIO_MOD_BANK_REG(0, RX_LDO0, LDO11_RFN_EN, 0x1);
                RADIO_MOD_BANK_REG(0, RX_LDO1, LDO11_RFS_EN, 0x1);
        }

        safety_monitor_mode = false;
        rf_loopback_mode = false;

        /* restore rx lobuff and lna bias setting */
        for (ch = 0; ch < 4; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, RLB_BI_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, LNA2_BI_EN, 0x1);
        }
        /* resume rxlo voltage selection */
        RADIO_MOD_BANK_REG(0, LO_LDO3, LDO11_RXLO_VO_SEL, old_rxlo_ldo);

        /* restore rx gain setting */
        for (ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, cfg->rf_tia_gain);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA2_GAINSEL, cfg->rf_vga2_gain);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN1, VGA1_GAINSEL, cfg->rf_vga1_gain);
        }
        /* restore agc setting */
        if(agc_sta)
        {
                // fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
                // RADIO_MOD_BANK_REG(5,FMCW_AGC_EN_1,AGC_EN_1,1);
                fmcw_radio_agc_enable(radio,true);
        }
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used for fault injection of Safety Monitor13: Chirp Monitor */
uint8_t fmcw_radio_sm_chirp_monitor_fault_injection_IRQ(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t IRQ_value = 0;

        fmcw_radio_sm_chirp_monitor_setting(radio);

        /* generate FMCW */
        RADIO_MOD_BANK_REG(0, FP_LDO3, LDO11_MD_EN, 0x1);
        fmcw_radio_generate_fmcw(radio);

        /* fault injection */
        // fmcw_radio_switch_bank(radio, 3);
        // RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x0);
        fmcw_radio_sm_chirp_monitor_fault_injection(radio, true);
        // EMBARC_PRINTF("reached sm chirp monitor fault injection\n");
        fmcw_radio_start_fmcw(radio);
        // MDELAY(1);
        UDELAY(500);

        /* read IRQ */
        fmcw_radio_switch_bank(radio, 10);
        IRQ_value = RADIO_READ_BANK_REG(10,ITF_IRQ_4);

        /* fault injection restore and clear IRQ status for monitoring */
        fmcw_radio_sm_chirp_monitor_fault_injection(radio, false);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x1);
        // UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x0);

        fmcw_radio_sm_chirp_monitor_IRQ_resume(radio);
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

uint8_t fmcw_radio_sm_chirp_monitor_IRQ(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t IRQ_value, ps_state_L, ps_state_M, ps_state_H, ps_state_INT;
        uint32_t random_seeds;
        baseband_t *bb = baseband_get_bb(0);
        baseband_hw_t *bb_hw = &bb->bb_hw;

        /* turn off TX */
#if AUTO_TX == 1
        fmcw_radio_tx_auto_ch_on(radio, -1, false);
#endif
        fmcw_radio_tx_ch_on(radio, -1, false);
        /* set chirp number and phase scramble init state*/
        fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_0, 1);
        fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_1, 0);
        /* enable chirp monitor */
        fmcw_radio_sm_chirp_monitor_setting(radio);
        /* Generate FMCW */
        fmcw_radio_start_fmcw(radio);
        UDELAY(500);
        /* Read IRQ */
        fmcw_radio_switch_bank(radio, 10);
        IRQ_value = RADIO_READ_BANK_REG(10,ITF_IRQ_4);
        /* resume chirp_monitor settings */
        fmcw_radio_sm_chirp_monitor_IRQ_resume(radio);

        /* resume FMCW settings */
        fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
        fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_0,   REG_L(radio->nchirp));
        fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_1,   REG_M(radio->nchirp));

        /* set BB the same logic seeds as FMCW */
        random_seeds = fmcw_radio_random_num_generation(radio);
        //EMBARC_PRINTF("random_seeds = %d\n", random_seeds);
        ps_state_L =   random_seeds & 0xff;
        ps_state_M =   (random_seeds >> 8) & 0xff;
        ps_state_H =   (random_seeds >> 16) & 0xff;
        ps_state_INT = (random_seeds >> 24) & 0xff;
        fmcw_radio_switch_bank(radio, 3);
        RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE0, ps_state_L);
        RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE1, ps_state_M);
        RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE2, ps_state_H);
        RADIO_WRITE_BANK_REG(3, FMCW_PS_STATE3, ps_state_INT);
        BB_WRITE_REG(bb_hw, FFT_DINT_DAT_PS, random_seeds);
        BB_WRITE_REG(bb_hw, SAM_DINT_DAT,    random_seeds);

        /* resume tx */
        fmcw_radio_tx_restore(radio);
#if AUTO_TX == 1
        fmcw_radio_tx_auto_ch_on(radio, -1, true);
#endif

        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}


/* This function is used for getting IRQ value of Safety Monitor14: Over Temperature Detector */
uint8_t fmcw_radio_sm_over_temp_detector_IRQ(fmcw_radio_t *radio, bool fault_injection)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        uint8_t IRQ_value;

        /* get over_temp_detector threahold */
        fmcw_radio_sm_over_temp_detector_threshold(radio, fault_injection);
        /* get over_temp_detector setting */
        fmcw_radio_sm_over_temp_detector_setting(radio);
        /* Read IRQ */
        IRQ_value = RADIO_READ_BANK_REG(10,ITF_IRQ_5);

        /* restore over_temp_detector threahold and clear IRQ status for monitoring */
        if (fault_injection) {
                fmcw_radio_sm_over_temp_detector_threshold(radio, false);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, ITF_TEMP_CLEAR, 0x1);
                UDELAY(100);
                RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, ITF_TEMP_CLEAR, 0x0);
        }

        /* Disable 5M ADC Clock */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        /* resume over_temp_detector_settings */
        fmcw_radio_sm_over_temp_detector_IRQ_resume(radio);
        /* enable avdd33 monitor */
#if (SAFETY_FEATURE_SM2 == 1)
        fmcw_radio_sm_avdd33_monitor_IRQ(radio, false);
#endif
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used to check tx on channels */
uint32_t fmcw_radio_check_txon_channel(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        uint8_t ch0_status, ch1_status, ch2_status, ch3_status;
        uint32_t ch;

        ch0_status = RADIO_READ_BANK_REG_FIELD(1, CH0_TX_EN0,PADR_BI_EN);
        ch1_status = RADIO_READ_BANK_REG_FIELD(1, CH1_TX_EN0,PADR_BI_EN);
        ch2_status = RADIO_READ_BANK_REG_FIELD(1, CH2_TX_EN0,PADR_BI_EN);
        ch3_status = RADIO_READ_BANK_REG_FIELD(1, CH3_TX_EN0,PADR_BI_EN);
        if (ch0_status) {
                ch = 0;
        } else if (ch1_status) {
                ch = 1;
        } else if (ch2_status) {
                ch = 2;
        } else if (ch3_status) {
                ch = 3;
        } else {
                EMBARC_PRINTF("no tx channel is on");
                ch = 4;
        }
        fmcw_radio_switch_bank(radio, old_bank);
        return ch;
}

 /* This function is used to calculate pa dout of pdt */
uint32_t fmcw_get_pa_dout(fmcw_radio_t *radio, int32_t channel_index, double freq, double power_th)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        /* calculate according to pdt formula */
        uint32_t Dconst, Dconst1, calon_dout, caloff_dout, paon_dout, paoff_dout;
        double meas_err = 0;
        power_th = power_th - meas_err;

        if (freq <= 78){
                Dconst1 = pow(10,(power_th-9.5)/12);
        }else{
                Dconst1 = pow(10,(power_th-7.5)/12);
        }

        /* set threashold for monitoring */
        /* cal on */
        fmcw_radio_pdt_reg(radio, 0, channel_index);
        fmcw_radio_auxadc2_voltage(radio,4);
        calon_dout = fmcw_radio_auxadc2_dout(radio); //zhonglei:could decrease delay time
        EMBARC_PRINTF("cal_on = %d\n", calon_dout);
        /* cal off */
        fmcw_radio_pdt_reg(radio, 1, channel_index);
        fmcw_radio_auxadc2_voltage(radio,4);
        caloff_dout = fmcw_radio_auxadc2_dout(radio);//zhonglei:could decrease delay time
        EMBARC_PRINTF("cal_off = %d\n", caloff_dout);
        Dconst = (calon_dout - caloff_dout) * Dconst1;
        EMBARC_PRINTF("cal_on = %d, cal_off = %d, cal_on-cal_off * const = %d\n", calon_dout, caloff_dout, Dconst);

        /* make dis = 1 before calculate pa_off*/
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, FP_EN, PFD_DL_DIS, 0x1);
        /* get pdt pa off reg */
        fmcw_radio_pdt_reg(radio, 3, channel_index);
        paoff_dout = fmcw_radio_auxadc2_dout(radio);
        /* after pa off, enable vco and make dis = 0*/
        RADIO_MOD_BANK_REG(0, FP_EN, VC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_EN, PFD_DL_DIS, 0x0);
        paon_dout = Dconst + paoff_dout;
        //hzl: could delete start
        fmcw_radio_pdt_reg(radio, 2, channel_index);
        fmcw_radio_auxadc2_voltage(radio,4);
        uint32_t pa_d = fmcw_radio_auxadc2_dout(radio);
        EMBARC_PRINTF("pa_dout=%d, paoff_dout=%d\n",pa_d,paoff_dout);
        /* disable 5M ADC clock */
        fmcw_radio_switch_bank(radio, old_bank);
        //hzl: could delete end
        return paon_dout;
}

/* This function is used for fault injection of Safety Monitor Power ON Self Test
 * fault_injection = true: enter power on self test
 * fault_injection = false: leave power on self test/normal monitoring
 */
void fmcw_radio_sm_fault_injection(fmcw_radio_t *radio){
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t IRQ_value;
        uint8_t IRQ_all = 0xFF;

        /* sm_rfpower_detector fault injection */
        IRQ_value = fmcw_radio_sm_rfpower_detector_fault_injection_IRQ(radio,5);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm RF Power Detector IRQ = %d\n",IRQ_value);

        /* sm_if_loopback fault injection */
        IRQ_value = fmcw_radio_sm_if_loopback_IRQ(radio,true);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm IF Loopback IRQ = %d\n",IRQ_value);

        /* sm_rf_loopback fault injection */
        IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(radio,true);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm RF Loopback IRQ = %d\n",IRQ_value);

        /* sm_ldo_monitor fault injection */
        IRQ_value = fmcw_radio_sm_ldo_monitor_fault_injection_IRQ(radio);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm LDO Monitor IRQ = %d\n",IRQ_value);

        /* sm_avdd33_monitor fault injection */
        IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(radio,true);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm AVDD33 Monitor IRQ = %d\n",IRQ_value);

        /* sm_dvdd11_monitor fault injection */
        IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(radio,true);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm DV11 Monitor IRQ = %d\n",IRQ_value);

        /* sm_bg_monitor fault injection */
        IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(radio,true);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm BG Monitor IRQ = %d\n",IRQ_value);

        /* sm_chirp_monitor fault injection */
        IRQ_value = fmcw_radio_sm_chirp_monitor_fault_injection_IRQ(radio);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm Chirp Monitor IRQ = %d\n",IRQ_value);

        /* sm_over_temp_detector fault injection */
        IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(radio,true);
        IRQ_all &= IRQ_value;
        EMBARC_PRINTF("sm Over Temp Detector IRQ = %d\n",IRQ_value);

        /* sm_saturation_detector fault injection */
        IRQ_value = fmcw_radio_sm_saturation_detector_fault_injection_IRQ(radio);
        if (IRQ_value){
                IRQ_all &= 0x01;
        }else{
                IRQ_all &= 0x00;
        }
        EMBARC_PRINTF("sm Saturation Detector IRQ = %d\n",IRQ_value);

        fmcw_radio_switch_bank(radio, old_bank);

        if (IRQ_all){
                EMBARC_PRINTF("/*** sm self-check done: IRQ_all = %d  success!***/\n\r", IRQ_all);
        }else{
                EMBARC_PRINTF("/*** sm self-check done: IRQ_all = %d  failed!***/\n\r", IRQ_all);
        }
}

/* This function is a basic setting of Safety Monitor1: LDO Monitor */
void fmcw_radio_sm_ldo_monitor_setting(fmcw_radio_t *radio, uint8_t ldo_part)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Enable AUXADC1 Test Point */
        fmcw_radio_auxadc1_voltage(radio,4);

#if (SAFETY_FEATURE_SM2 == 1)
        /* Clear IRQ of LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
        /* Disable AVDD33 Monitor */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_RDAC_EN, 0x0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_EN, 0x0);
#endif
        /* Enable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x1);
        /* Enable auxadc1 */
        RADIO_WRITE_BANK_REG(1, DTSDM1_EN, 0x7e);
        /* Enable LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        switch (ldo_part) {
        case 0:
                /* ldo25 */
                fmcw_radio_switch_bank(radio, 2);
                RADIO_MOD_BANK_REG(2, LVDS_LDO25, LDO25_LVDS_LDO_EN, 0x1);
                fmcw_radio_switch_bank(radio, 10);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_0, 0xFA);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_1, 0x03);
                break;
        case 1:
                /* ldo25_adc01 & ldo12/11_adc & rxrf & spi */
                fmcw_radio_switch_bank(radio, 10);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_1, 0x1C);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_2, 0xD0);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_3, 0x80);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_4, 0x01);
                break;
        case 2:
                /* ldo11 fmcwpll & lo */
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_1, 0x20);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_3, 0x37);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_4, 0x42);
                break;
        case 3:
                /* ldo11_tx & ldo11_tx_pa */
                fmcw_radio_switch_bank(radio, 1);
                RADIO_WRITE_BANK_REG(1, TX_LDO_EN, 0xFF);
                fmcw_radio_switch_bank(radio, 10);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_2, 0x0F);
                RADIO_WRITE_BANK_REG(10, LDO_MON_EN_4, 0x3C);
                break;
        default:
                EMBARC_PRINTF("\n\n");
                break;
        }

        /* enable ldo monitor */
        RADIO_WRITE_BANK_REG(10, LDO_MON_RSTN, 0x1);

        /* clear IRQ of ldo monitor */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_CLEAR, 0x1);
        UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_CLEAR, 0x0);

        /* start ldo monitor */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_T_START, 0x1);
        UDELAY(300);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_LDO_T_START, 0x0);

#if (SAFETY_FEATURE_SM2 == 1)
        fmcw_radio_sm_avdd33_monitor_IRQ(radio, false);
#endif
        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is threashold setting for Safety Monitor1: LDO Monitor */
void fmcw_radio_sm_ldo_monitor_threshold(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);

        /* Set threshold of 1V LDO Monitor */
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH10150_LOW_0,  0xE0);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH10150_LOW_1,  0x83);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH10150_LOW_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1075_LOW_0,   0xE0);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1075_LOW_1,   0x83);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1075_LOW_2,   0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1075_HIGH_0,  0x6C);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1075_HIGH_1,  0xB2);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1075_HIGH_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH10150_HIGH_0, 0x6C);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH10150_HIGH_1, 0xB2);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH10150_HIGH_2, 0x1);

        /* Set threshold of 1.1V LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH11150_LOW_0,  0x11);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH11150_LOW_1,  0x91);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH11150_LOW_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1175_LOW_0,   0xDD);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1175_LOW_1,   0x9D);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1175_LOW_2,   0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1175_HIGH_0,  0x77);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1175_HIGH_1,  0xB7);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1175_HIGH_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH11150_HIGH_0, 0x44);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH11150_HIGH_1, 0xC4);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH11150_HIGH_2, 0x1);

        /* Set threshold of 1.15V LDO Monitor */
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH12150_LOW_0,  0xA9);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH12150_LOW_1,  0x97);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH12150_LOW_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1275_LOW_0,   0x0B);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1275_LOW_1,   0xA5);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1275_LOW_2,   0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1275_HIGH_0,  0xCE);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1275_HIGH_1,  0xBF);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1275_HIGH_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH12150_HIGH_0, 0x30);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH12150_HIGH_1, 0xCD);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH12150_HIGH_2, 0x1);

        /* Set threshold of 1.3V LDO Monitor */
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH13150_LOW_0,  0x71);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH13150_LOW_1,  0xAB);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH13150_LOW_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1375_LOW_0,   0x91);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1375_LOW_1,   0xBA);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1375_LOW_2,   0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1375_HIGH_0,  0xD3);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1375_HIGH_1,  0xD8);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH1375_HIGH_2,  0x1);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH13150_HIGH_0, 0xF3);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH13150_HIGH_1, 0xE7);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH13150_HIGH_2, 0x1);

        /* Set threshold of 2.5V LDO Monitor */
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH25150_LOW_0,  0xB2);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH25150_LOW_1,  0x49);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH25150_LOW_2,  0x2);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH2575_LOW_0,   0x7C);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH2575_LOW_1,   0x70);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH2575_LOW_2,   0x2);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH2575_HIGH_0,  0x45);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH2575_HIGH_1,  0x97);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH2575_HIGH_2,  0x2);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH25150_HIGH_0, 0x0F);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH25150_HIGH_1, 0xBE);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTH25150_HIGH_2, 0x2);

        /* set monitoring threshold */
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTHSEL, 0x1F);
        fmcw_radio_switch_bank(radio, old_bank);

}

void fmcw_radio_sm_ldo_monitor_IRQ_resume(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        /* disable 5M AUXADC CLK & auxadc1 selection */
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, DTSDM_MUXIN_SEL, 0x0);
        /* disable auxadc1 */
        RADIO_WRITE_BANK_REG(1, DTSDM1_EN, 0x00);
        /* disable LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_WRITE_BANK_REG(10, LDO_MON_EN_0, 0x00);
        RADIO_WRITE_BANK_REG(10, LDO_MON_EN_1, 0x00);
        RADIO_WRITE_BANK_REG(10, LDO_MON_EN_2, 0x00);
        RADIO_WRITE_BANK_REG(10, LDO_MON_EN_3, 0x00);
        RADIO_WRITE_BANK_REG(10, LDO_MON_EN_4, 0x00);
        RADIO_WRITE_BANK_REG(10, LDO_MON_RSTN, 0x0);
        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is a basic setting of Safety Monitor2: AVDD33 Monitor */
void fmcw_radio_sm_avdd33_monitor_setting(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Enable safety monitor LDO */
        RADIO_MOD_BANK_REG(0, LDO25_SM, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO11_SM, LDO_EN, 0x1);

        /* Enable AVDD33 Monitor */
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_RDAC_EN, 0x1);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_EN, 0x1);

        /* Clear IRQ of AVDD33 Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x0);

        /* Start AVDD33 Monitor */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_T_START, 0x1);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_T_START, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is threashold setting for Safety Monitor2: AVDD33 Monitor
 * fault_injection = true: enter power on self test
 * fault_injection = false: leave power on self test/normal monitoring
 */
void fmcw_radio_sm_avdd33_monitor_threshold(fmcw_radio_t *radio, bool fault_injection){
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* power on self test */
        if (fault_injection){
                EMBARC_PRINTF("reached sm avdd33 monitor fault injection\n");
                /* Set threshold of AVDD33 LDO Monitor as 0 */
                RADIO_WRITE_BANK_REG(0, CBC33_MON0, 0x00);
                RADIO_WRITE_BANK_REG(0, CBC33_MON1, 0x00);
        }

        else{
                /* Set threshold of AVDD33 Monitor */
                RADIO_WRITE_BANK_REG(0, CBC33_MON0, 0x43);
                RADIO_WRITE_BANK_REG(0, CBC33_MON1, 0xBA);
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is a basic setting of Safety Monitor3: DV11 Monitor */
void fmcw_radio_sm_dvdd11_monitor_setting(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Enable safety monitor DV11 */
        RADIO_MOD_BANK_REG(0, LDO25_SM, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO11_SM, LDO_EN, 0x1);

        /* Enable DV11 Monitor */
        RADIO_MOD_BANK_REG(0, T_CBC2, DV11_MON_EN, 0x1);

        /* Clear IRQ of DV11 Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_DV11_CLEAR, 0x1);
        // UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_DV11_CLEAR, 0x0);

        /* Start DV11 Monitor */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_DV11_T_START, 0x1);
        // UDELAY(300);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_DV11_T_START, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is threashold setting for Safety Monitor3: DV11 Monitor
 * fault_injection = true: enter power on self test
 * fault_injection = false: leave power on self test/normal monitoring
 */
void fmcw_radio_sm_dvdd11_monitor_threshold(fmcw_radio_t *radio, bool fault_injection){
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* power on self test */
        if (fault_injection){
                EMBARC_PRINTF("reached sm dvdd11 monitor fault injection\n");
                RADIO_MOD_BANK_REG(0, DV11_MON, DV11_MON_VHSEL, 0x0);
                RADIO_MOD_BANK_REG(0, DV11_MON, DV11_MON_VLSEL, 0x0);
        }

        else{
                /* Set threshold of DV11 Monitor */
                RADIO_MOD_BANK_REG(0, DV11_MON, DV11_MON_VHSEL, 0x1);
                RADIO_MOD_BANK_REG(0, DV11_MON, DV11_MON_VLSEL, 0x1);
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is a basic setting of Safety Monitor4: Bandgap Voltage Monitor */
void fmcw_radio_sm_bg_monitor_setting(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Enable safety monitor BG */
        RADIO_MOD_BANK_REG(0, LDO25_SM, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO11_SM, LDO_EN, 0x1);

        /* Enable BG Monitor */
        RADIO_MOD_BANK_REG(0, T_CBC2, VBG_MON_EN, 0x1);

        /* Clear IRQ of BG Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_VBG_CLEAR, 0x1);
        // UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_VBG_CLEAR, 0x0);

        /* Start BG Monitor */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_VBG_T_START, 0x1);
        // UDELAY(300);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_VBG_T_START, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is threashold setting for Safety Monitor4: BG Monitor
 * fault_injection = true: enter power on self test
 * fault_injection = false: leave power on self test/normal monitoring
 */
void fmcw_radio_sm_bg_monitor_threshold(fmcw_radio_t *radio, bool fault_injection){
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* power on self test */
        if (fault_injection){
                EMBARC_PRINTF("reached sm bg monitor fault injection\n");
                RADIO_MOD_BANK_REG(0, VBG_MON, VBG_MON_VHSEL, 0x0);
                RADIO_MOD_BANK_REG(0, VBG_MON, VBG_MON_VLSEL, 0x0);
        }

        else{
                /* Set threshold of BG Monitor */
                RADIO_MOD_BANK_REG(0, VBG_MON, VBG_MON_VHSEL, 0x1);
                RADIO_MOD_BANK_REG(0, VBG_MON, VBG_MON_VLSEL, 0x1);
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/*
 * This function is a basic setting of Safety Monitor: RF Power Detector
 * freq            : lock freq
 * power_th        : rf power lower threahold
 * channel_index   : 0/ 1/ 2/ 3
 */
void fmcw_radio_sm_rfpower_detector_setting(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        fmcw_radio_auxadc2_voltage(radio, 4);
#if (SAFETY_FEATURE_SM2 == 1)
        /* Clear IRQ of LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
        /* Disable AVDD33 Monitor */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_RDAC_EN, 0x0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_EN, 0x0);
#endif
        /* enable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x1);
        /* enable auxadc2 */
        RADIO_WRITE_BANK_REG(1, DTSDM2_EN, 0x7e);
        /* set threshold of RF power detector timing */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TIME_CNT_1, 0x3F);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_CTU_SIZE, 0x02);

        /* enable RF power detector */
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_RSTN, 0x1);

        /* enable TX monitor according to channel_index */
        fmcw_radio_switch_bank(radio, 10);
        switch (channel_index) {
        case 0:
                RADIO_MOD_BANK_REG(10, RFPOWER_MON_EN_1, TX0_EN, 0x1);
                break;
        case 1:
                RADIO_MOD_BANK_REG(10, RFPOWER_MON_EN_0, TX1_EN, 0x1);
                break;
        case 2:
                RADIO_MOD_BANK_REG(10, RFPOWER_MON_EN_0, TX2_EN, 0x1);
                break;
        case 3:
                RADIO_MOD_BANK_REG(10, RFPOWER_MON_EN_0, TX3_EN, 0x1);
                break;
        default:
                EMBARC_PRINTF("\n\n");
                break;
        }

        /* clear IRQ of rf power detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_CLEAR, 0x1);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_CLEAR, 0x0);
        /* start rf power detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_T_START, 0x1);
        MDELAY(1);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_T_START, 0x0);
        /* disable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        /* disable auxadc2 */
        RADIO_WRITE_BANK_REG(1, DTSDM2_EN, 0x00);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is threashold setting for Fault Injection of Safety Monitor: RF Power Detector */
void fmcw_radio_sm_rfpower_detector_fault_injection_threshold(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);

        /* set threashold for power on self test */
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TXVTH_LOW_2, 0x3);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is threashold setting for Safety Monitor: RF Power Detector */
void fmcw_radio_sm_rfpower_detector_threshold(fmcw_radio_t *radio, double freq, double power_th, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        uint32_t pa_on, doutL, doutM, doutH;

        /* calculate pa on code */
        pa_on = floor(fmcw_get_pa_dout(radio, channel_index, freq, power_th));
        EMBARC_PRINTF("pa_on threshold = %d\n", pa_on);
        doutH = pa_on >> 16;
        doutM = (pa_on >> 8) - (doutH << 8);
        doutL = pa_on - (doutM << 8) - (doutH << 16) ;
        /* set voltage low threashold */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TXVTH_LOW_0, doutL);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TXVTH_LOW_1, doutM);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TXVTH_LOW_2, doutH);
        /* set voltage high threashold */
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TXVTH_HIGH_0, 0x00);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TXVTH_HIGH_1, 0x7C);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TXVTH_HIGH_2, 0x03);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is to resume all settings for Safety Monitor: RF Power Detector */
void fmcw_radio_sm_rf_power_detector_IRQ_resume(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        /* clear IRQ of rf power detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_CLEAR, 0x1);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_2, ITF_RFPOWER_CLEAR, 0x0);
        /* resume threshold of RF power detector timing */
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_TIME_CNT_1, 0x01);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_CTU_SIZE, 0x80);

        /* disable RF power detector */
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_RSTN, 0x0);

        /* disable TX monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_EN_1, 0x0);
        RADIO_WRITE_BANK_REG(10, RFPOWER_MON_EN_0, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is used for resuming register setting of power detector */
void fmcw_radio_pdt_reg_resume(fmcw_radio_t *radio, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);

        /* resume pdt and tpana setting */
        RADIO_WRITE_BANK_CH_REG(1, channel_index, TX_TN2, 0x0);
        fmcw_radio_switch_bank(radio, 0);
        RADIO_WRITE_BANK_REG(0, TPANA1, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is used for resuming register setting of rf loopback */
void fmcw_radio_rf_loopback_reg_resume(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t ch;

        /* resume lna2_bias_en and rxlobuff_bias_en */
        for (ch = 0; ch < 4; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, RLB_BI_EN, 0x1);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_RF_EN, LNA2_BI_EN, 0x1);
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is a basic setting of Safety Monitor13: Chirp Monitor */
void fmcw_radio_sm_chirp_monitor_setting(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* enable Chirp Monitor */
        RADIO_MOD_BANK_REG(0, FP_LDO4, LDO11_CM_EN, 0x1);
        RADIO_MOD_BANK_REG(0, FP_LDO4, LDO11_CM_VO_SEL, 0x7);
        RADIO_MOD_BANK_REG(0, POR, LDO11_SPI_VO_SEL, 0x7);
        RADIO_MOD_BANK_REG(0, FP_EN, CM_EN, 0x1);

        /* set chirp monitor threshold */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, FM_CTRL_1, FM_DIFFERS_TH, 0x6);
        RADIO_MOD_BANK_REG(10, FM_CTRL_1, FM_DIFFERS_CNTSIZE, 0x1F);

        /* enable Chirp Monitor IRQ */
        RADIO_MOD_BANK_REG(10, FM_CTRL_0, FM_T_EN, 0x1);
        RADIO_MOD_BANK_REG(10, FM_CTRL_0, FM_RSTN, 0x1);

        /* Reset Release for IRQ of Chirp Detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, FM_IRQ_RSTN, 0x0);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, FM_IRQ_RSTN, 0x1);

        /* Clear IRQ of Chirp Detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x1);
        // UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x0);

        /* Start IRQ of Chirp Detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_T_START, 0x1);
        // UDELAY(300);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_T_START, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
}


/* This function is to resume setting of Safety Monitor13: Chirp Monitor */
void fmcw_radio_sm_chirp_monitor_IRQ_resume(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        /* Clear IRQ of Chirp Detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x1);
        // UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x0);
        /* disable Chirp Monitor IRQ */
        RADIO_MOD_BANK_REG(10, FM_CTRL_0, FM_RSTN, 0x0);
        RADIO_MOD_BANK_REG(10, FM_CTRL_0, FM_T_EN, 0x0);
        /* disable Chirp Monitor */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, FP_EN, CM_EN, 0x0);
        /* resume ldo */
        RADIO_MOD_BANK_REG(0, FP_LDO4, LDO11_CM_VO_SEL, 0x4);
        RADIO_MOD_BANK_REG(0, POR, LDO11_SPI_VO_SEL, 0x4);
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_sm_chirp_monitor_fault_injection(fmcw_radio_t *radio, bool fault_injection)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 3);
        /* power on self test */
        if (fault_injection){
                RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x0);
                EMBARC_PRINTF("reached sm chirp monitor fault injection\n");
        }
        /* fault injection restore */
        else{
                RADIO_MOD_BANK_REG(3, FMCW_START, RSTN_SDM_MASH, 0x1);
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is a basic setting of Safety Monitor14: Over Temperature Detector */
void fmcw_radio_sm_over_temp_detector_setting(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, TEMP_IRQ_RSTN, 0x0);
        /* get temperature */
        fmcw_radio_get_temperature(radio);
#if (SAFETY_FEATURE_SM2 == 1)
        /* Clear IRQ of LDO Monitor */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_1, ITF_SUPPLY_CBC33_CLEAR, 0x1);
        /* Disable AVDD33 Monitor */
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_RDAC_EN, 0x0);
        RADIO_MOD_BANK_REG(0, T_CBC2, CBC33_MON_EN, 0x0);
#endif
        /* enable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x1);
        /* enable auxadc2 */
        RADIO_WRITE_BANK_REG(1, DTSDM2_EN, 0x7e);
        /* Enable Over Temperature Detector */
        fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, TEMP_IRQ_RSTN, 0x1);
        /* Clear IRQ of Over Temp Detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, ITF_TEMP_CLEAR, 0x1);
        UDELAY(100);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, ITF_TEMP_CLEAR, 0x0);
        /* Trigger rising edge of Over Temp Detector */
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, ITF_TEMP_T_START, 0x1);
        UDELAY(300);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, ITF_TEMP_T_START, 0x0);
        /* disable 5M AUXADC CLK */
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, DTSMD1_MUXIN_SEL, ADC_CLK_5M_EN, 0x0);
        /* disable auxadc2 */
        RADIO_WRITE_BANK_REG(1, DTSDM2_EN, 0x00);

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function is the threashold setting of Safety Monitor: Over Temperature Detector */
void fmcw_radio_sm_over_temp_detector_threshold(fmcw_radio_t *radio, bool fault_injection)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        /* power on self test */
        if (fault_injection){
                /* set 125C AUXADC2 Ref Code as 0 to Threadshold */
                RADIO_WRITE_BANK_REG(10, TEMP_MON_VTH_HIGH_2, 0x00);
                RADIO_WRITE_BANK_REG(10, TEMP_MON_VTH_HIGH_1, 0x00);
                RADIO_WRITE_BANK_REG(10, TEMP_MON_VTH_HIGH_0, 0x00);
                EMBARC_PRINTF("reached sm over temperature detector fault injection\n");
        }

        else{
                /* send 125C AUXADC2 Ref Code 0x01A61A to Threadshold */
                RADIO_WRITE_BANK_REG(10, TEMP_MON_VTH_HIGH_2, 0x01);
                RADIO_WRITE_BANK_REG(10, TEMP_MON_VTH_HIGH_1, 0xA6);
                RADIO_WRITE_BANK_REG(10, TEMP_MON_VTH_HIGH_0, 0x1A);
        }

        fmcw_radio_switch_bank(radio, old_bank);
}


void fmcw_radio_sm_over_temp_detector_IRQ_resume(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 10);
        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_5, TEMP_IRQ_RSTN, 0x0);
        fmcw_radio_switch_bank(radio, old_bank);
}

uint32_t fmcw_radio_sm_saturation_detector_fault_injection_IRQ(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint32_t IRQ_value=0;
        /* read back the bank selected in RTL */
        baseband_t* bb = baseband_get_rtl_frame_type();
        sensor_config_t* cfg = (sensor_config_t*)(bb->cfg);
        baseband_hw_t* bb_hw = &bb->bb_hw;
        /* calib times */
        int calib_cnt = 1;

        /* read old agc parameter */
        uint8_t old_agc_mode = cfg->agc_mode;
        float old_agc_tia_thres = cfg->agc_tia_thres;
        float old_agc_vga1_thres = cfg->agc_vga1_thres;
        float old_agc_vga2_thres = cfg->agc_vga2_thres;
        uint8_t old_rf_tia_gain = cfg->rf_tia_gain;
        uint8_t old_rf_vga1_gain = cfg->rf_vga1_gain;
        uint8_t old_rf_vga2_gain = cfg->rf_vga2_gain;
        uint8_t old_rf_lna_gain = RADIO_READ_BANK_CH_REG_FIELD(0, 0, RX_TN0, LNA_GC);
        uint8_t old_rf_bb_vga2 = RADIO_READ_BANK_CH_REG_FIELD(0, 0, RX_PDT, PKD_VGA);
        uint8_t old_rf_bb_vga1 = RADIO_READ_BANK_CH_REG_FIELD(0, 0, RX_PDT, PKD_EN);
        uint8_t old_rf_tia_sat = RADIO_READ_BANK_CH_REG_FIELD(0, 0, RX_PDT, SAT_EN);

        /* agc fault injection parameter setting */
        cfg->agc_mode = 2;
        cfg->agc_tia_thres = 0;
        cfg->agc_vga1_thres = 0;
        cfg->agc_vga2_thres = 0;
        cfg->rf_tia_gain = 4;
        cfg->rf_vga1_gain = 6;
        cfg->rf_vga2_gain = 6;
        /* agc_mode=2,agc_tia_thres = 0,agc_vga1_thres = 0,cfg->agc_vga2_thres = 0 */
        baseband_agc_init(bb_hw, cfg->agc_mode);
        /* baseband dump init */
        uint32_t old_buf_store = baseband_switch_buf_store(bb_hw, SYS_BUF_STORE_ADC);
        uint8_t old_bnk = BB_READ_REG(bb_hw, FDB_SYS_BNK_ACT); // read back the bank selected in RTl
        uint16_t old_status_en = BB_READ_REG(bb_hw, SYS_ENABLE);

        while(1){
                if(calib_cnt > 0){
                        /* start baseband */
                        baseband_start_with_params(bb, true, true,
                                        ((1 << SYS_ENABLE_SAM_SHIFT) | (SYS_ENABLE_AGC_MASK << SYS_ENABLE_AGC_SHIFT)),
                                        true, BB_IRQ_DISABLE_ALL, false);
                        /* wait done */
                        BASEBAND_ASSERT(baseband_wait_bb_done(POL_MODE, DEFAULT_TIMOUT) == E_OK);

                        /* Search test target peak in 2D-FFT plane */
                        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);

                        IRQ_value = BB_READ_REG(NULL, AGC_IRQ_STATUS);
                        EMBARC_PRINTF("AGC_IRQ_Value = 0x%x \n", IRQ_value);
                        /* restore tx parameter */
                        /* fmcw_radio_tx_restore(&bb->radio); */
                        baseband_switch_mem_access(bb_hw, old);
                        calib_cnt--;
                }else if (calib_cnt == 0){
                        break;
                }else{
                        taskYIELD();
                }
        }

        /* recover agc default parameter */
        fmcw_radio_switch_bank(radio, 5);
        RADIO_MOD_BANK_REG(5, FMCW_AGC_EN_1, AGC_EN_1,0x0);
        cfg->agc_mode = old_agc_mode;
        cfg->agc_tia_thres = old_agc_tia_thres;
        cfg->agc_vga1_thres = old_agc_vga1_thres;
        cfg->agc_vga2_thres = old_agc_vga2_thres;
        cfg->rf_tia_gain = old_rf_tia_gain;
        cfg->rf_vga1_gain = old_rf_vga1_gain;
        cfg->rf_vga2_gain = old_rf_vga2_gain;
        fmcw_radio_switch_bank(radio, 0);
        for (uint8_t ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, LNA_GC, old_rf_lna_gain);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_VGA, old_rf_bb_vga2);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, PKD_EN, old_rf_bb_vga1);
                RADIO_MOD_BANK_CH_REG(0, ch, RX_PDT, SAT_EN, old_rf_tia_sat);
        }
        baseband_agc_init(bb_hw, cfg->agc_mode);
        EMBARC_PRINTF("agc_mode=%d tia_thres=%.2f vga1_thres=%.2f vga2_thres=%.2f tia_gain=%d vga1_gain=%d vga2_gain=%d \n", cfg->agc_mode, cfg->agc_tia_thres,cfg->agc_vga1_thres,cfg->agc_vga2_thres, cfg->rf_tia_gain,cfg->rf_vga1_gain, cfg->rf_vga2_gain);
        /* clear EMU RF saturate ERR_STA */
        BB_WRITE_REG(NULL, AGC_SAT_CNT_CLR_FRA,  1);
        BB_WRITE_REG(NULL, AGC_IRQ_CLR, 0xFFF);
        raw_writel(REG_EMU_RF_ERR_CLR, raw_readl(REG_EMU_RF_ERR_STA));

        BB_WRITE_REG(bb_hw, SYS_BNK_ACT, old_bnk);
        BB_WRITE_REG(bb_hw, SYS_ENABLE, old_status_en);
        baseband_switch_buf_store(bb_hw, old_buf_store);
        fmcw_radio_switch_bank(radio, old_bank);
        return IRQ_value;
}

/* This function is used for dout code calculation of auxadc2 */
uint32_t fmcw_radio_auxadc2_dout(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);

        /* read back AUXADC2 Filter Output Digital Bits */
        uint32_t doutL, doutM, doutH, dout;
        /* 1st bring up, cannot readback dout, delay is necessary */
        MDELAY(1);
        doutL = RADIO_READ_BANK_REG(1, DTSDM2_DAT0);
        doutM = RADIO_READ_BANK_REG(1, DTSDM2_DAT1);
        doutH = RADIO_READ_BANK_REG(1, DTSDM2_DAT2);
        dout = doutL + ( doutM << 8 ) + ( doutH << 16 );

        fmcw_radio_switch_bank(radio, old_bank);
        return dout;
}

/* generate a random number based on running time */
uint32_t fmcw_radio_random_num_generation(fmcw_radio_t *radio)
{
        uint32_t ticks = chip_get_cur_us();
        srand(ticks);
        return rand();
}
#endif // FUNC_SAFETY

/* radio 2.5V and 1.3V ldo under power save mode */
int32_t fmcw_radio_adc_ldo_on(fmcw_radio_t *radio, bool enable)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        int ch;

        /* set ADC reg in power save mode */
        for (ch = 0; ch < MAX_NUM_RX; ch++){
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, RST, 0x0);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, BUFFER_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, BUFFER_VCMBUF_EN, 0x0);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, ANALS_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, OP1_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, OP2_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN0, OP3_EN, enable);

                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, CMP_VCALREF_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, BI_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, IDAC1_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, IDAC3_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, ESL_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, REFPBUF_EN, enable);
                RADIO_MOD_BANK_CH_REG(1, ch, ADC_EN1, REFNBUF_EN, enable);
        }

        fmcw_radio_switch_bank(radio, old_bank);
        return E_OK;
}

/* heck tx groups configuration to disable txlo buf for power saving purpose */
void fmcw_radio_txlobuf_on(fmcw_radio_t *radio){
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* enable txlo buffer */
        RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG3_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG2_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG1_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG3_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG2_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG1_EN, 0x1);

        /* check if txlo buffer could be off */
        if ((cfg->tx_groups[0] == 0) && (cfg->tx_groups[1] == 0)){
                RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG3_EN, 0x0);
                RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG2_EN, 0x0);
                RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG1_EN, 0x0);
        }
        if ((cfg->tx_groups[2] == 0) && (cfg->tx_groups[3] == 0)){
                RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG3_EN, 0x0);
                RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG2_EN, 0x0);
                RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG1_EN, 0x0);
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

#if RF_COMP == 1
/* detailed definition */
#define COMP_ARRAY 10
#define LO_COMP 1
#define TX_COMP 1    /* tx compensation can only be used in TX_POWER_DEFAULT */
#define RX_COMP 1

/* detailed compensation function */
static void fmcw_radio_lo_comp_code(fmcw_radio_t *radio, uint8_t array_pos);
static void fmcw_radio_tx_comp_code(fmcw_radio_t *radio, uint8_t array_pos);
static void fmcw_radio_rx_comp_code(fmcw_radio_t *radio, uint8_t array_pos);

/* compensation code array */
static int lomain_code[COMP_ARRAY] = {2, 4, 4, 4, 6, 6, 6, 6, 6, 6};
static int txlo_code[COMP_ARRAY]   = {2, 4, 4, 5, 6, 6, 6, 6, 6, 6};
static int tx_code[COMP_ARRAY]     = {4, 4, 4, 4, 4, 4, 5, 5, 5, 5};
static int pa_code[COMP_ARRAY]     = {0x8, 0xa, 0xa, 0xa, 0xa, 0xa, 0xa, 0xc, 0xd, 0xd};
static int rxlo_code[COMP_ARRAY]   = {4, 4, 5, 6, 7, 7, 7, 7, 7, 7};
static int rx_code[COMP_ARRAY]     = {4, 4, 4, 4, 4, 5, 5, 5, 5, 5};

/* This function provide RF compensation code vs Junction Temp */
float fmcw_radio_rf_comp_code(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        float temp;
        int junc_temp;
        uint8_t array_pos;
        /* get temperature */
        temp = fmcw_radio_get_temperature(radio);
        /* round temperture to 10C/step */
        junc_temp = round(temp / 10.0) * 10;
        /* calculate compensation code position according to readback temperature */
        if (junc_temp <= 0) {
                array_pos = 0;
        } else if (junc_temp <= 50) {
                array_pos = 1;
        } else if (junc_temp <= 120) {
                array_pos = junc_temp/10 - 4;
        } else {
                array_pos = 9;
        }

        /* call compensation functions */
        if (LO_COMP) {
                fmcw_radio_lo_comp_code(radio, array_pos);
        }
        if (TX_COMP) {
                fmcw_radio_tx_comp_code(radio, array_pos);
        }
        if (RX_COMP) {
                fmcw_radio_rx_comp_code(radio, array_pos);
        }

        fmcw_radio_switch_bank(radio, old_bank);
        return temp;
}

/* This function provide LO compensation code vs Junction Temp */
void fmcw_radio_lo_comp_code(fmcw_radio_t *radio, uint8_t array_pos)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        /* apply compensation code */
        RADIO_MOD_BANK_REG(0, LO_LDO0, LDO11_LMP_VO_SEL, lomain_code[array_pos]);
        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function provide TX compensation code vs Junction Temp, tx compensation can only be used in TX_POWER_DEFAULT */
void fmcw_radio_tx_comp_code(fmcw_radio_t *radio, uint8_t array_pos)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t ch;

        /* apply compensation code */
        RADIO_MOD_BANK_REG(0, LO_LDO2, LDO11_TXLO_VO_SEL, txlo_code[array_pos]);
        fmcw_radio_switch_bank(radio, 1);
        RADIO_MOD_BANK_REG(1, TX_LDO0, LDO11_TX0_VO_SEL, tx_code[array_pos]);
        RADIO_MOD_BANK_REG(1, TX_LDO1, LDO11_TX1_VO_SEL, tx_code[array_pos]);
        RADIO_MOD_BANK_REG(1, TX_LDO2, LDO11_TX2_VO_SEL, tx_code[array_pos]);
        RADIO_MOD_BANK_REG(1, TX_LDO3, LDO11_TX3_VO_SEL, tx_code[array_pos]);
        for(ch = 0; ch < MAX_NUM_TX; ch++) {
                RADIO_MOD_BANK_CH_REG(1, ch, PA_LDO, LDO11_TX_PA_VO_SEL, pa_code[array_pos]);
        }
        /* apply pa bias code only when tx power equals to TX_POWER_DEFAULT */
        if (array_pos == 0) {
                for(ch = 0; ch < MAX_NUM_TX; ch++) {
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI0, PA_BI, 0x8);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI0, PADR_BI, 0x7);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI1, PA2_BI, 0x8);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI1, PDR_BI, 0x7);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI2, IDAC_BI, 0x7);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI2, QDAC_BI, 0x7);
                }
        } else {
                for(ch = 0; ch < MAX_NUM_TX; ch++) {
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI0, PA_BI, 0xa);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI0, PADR_BI, 0xa);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI1, PA2_BI, 0xa);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI1, PDR_BI, 0xa);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI2, IDAC_BI, 0x8);
                        RADIO_MOD_BANK_CH_REG(1, ch, TX_BI2, QDAC_BI, 0x8);
                }
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/* This function provide LO compensation code vs Junction Temp */
void fmcw_radio_rx_comp_code(fmcw_radio_t *radio, uint8_t array_pos)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        /* apply compensation code */
        RADIO_MOD_BANK_REG(0, LO_LDO3, LDO11_RXLO_VO_SEL, rxlo_code[array_pos]);
        RADIO_MOD_BANK_REG(0, RX_LDO0, LDO11_RFN_VO_SEL, rx_code[array_pos]);
        RADIO_MOD_BANK_REG(0, RX_LDO1, LDO11_RFS_VO_SEL, rx_code[array_pos]);
        fmcw_radio_switch_bank(radio, old_bank);
}
#endif

uint32_t* fmcw_doppler_move(fmcw_radio_t *radio)
{
        static uint32_t doppler_move_opt[4];
        uint32_t step_down_opt,down_cycle_opt,wait_cycle_opt,T_dn_test;
        uint32_t bits_frac = 28;
        uint32_t bits_frac_shift = 1 << bits_frac;
        uint32_t T_dn_opt = 0;
        uint32_t T_up = radio->up_cycle;
        uint32_t T_dn = radio->down_cycle;
        uint32_t T_idle = radio->cnt_wait;
        uint32_t T_dn_begin = T_dn;
        double FL_dec_final = (1.0 * radio->start_freq) / bits_frac_shift;
        double FH_dec_final = (1.0 * radio->stop_freq) / bits_frac_shift;
        double Fstep_up_dec_final = (1.0 * radio->step_up) / bits_frac_shift;
        double Fstep_dn_dec_final = (1.0 * radio->step_down) / bits_frac_shift;
        double cal_acc1, acc1, acc1_err;
        double err_min = 1;
        doppler_move_opt[0] = 0;

        /* get doppler spur acc1 value */
        cal_acc1 = (T_up) * FL_dec_final + (T_up-1) * (T_up) / 2 * Fstep_up_dec_final +
                   (T_dn) * FH_dec_final - (T_dn-1) * (T_dn) / 2 * Fstep_dn_dec_final +
                   FL_dec_final * (T_idle-T_dn);
        acc1 = cal_acc1 - floor(cal_acc1);

        for (T_dn_test = T_dn_begin; T_dn_test < T_idle; T_dn_test++) {
                Fstep_dn_dec_final = floor((FH_dec_final - FL_dec_final) / T_dn_test * bits_frac_shift) / bits_frac_shift;
                T_dn = ceil((FH_dec_final-FL_dec_final) / Fstep_dn_dec_final);

                /* Estimate doppler position */
                cal_acc1 = (T_up) * FL_dec_final + (T_up-1) * (T_up) / 2 * Fstep_up_dec_final +
                           (T_dn) * FH_dec_final - (T_dn-1) * (T_dn) / 2 * Fstep_dn_dec_final +
                           FL_dec_final * (T_idle-T_dn);
                acc1 = cal_acc1 - floor(cal_acc1);

                if(acc1<0.5) {
                        acc1_err = fabs(acc1-0.25);
                } else {
                        acc1_err = fabs(acc1-0.75);
                }

                if(acc1_err < err_min) {
                        err_min = acc1_err;
                        T_dn_opt = T_dn_test;
                        doppler_move_opt[0] = 1;
                }
        }

        Fstep_dn_dec_final = floor((FH_dec_final - FL_dec_final) / T_dn_opt* bits_frac_shift) / bits_frac_shift;
        step_down_opt =  floor(Fstep_dn_dec_final * bits_frac_shift);
        T_dn = ceil((FH_dec_final-FL_dec_final) / Fstep_dn_dec_final);
        down_cycle_opt = T_dn;
        wait_cycle_opt = radio->cnt_wait - down_cycle_opt;

        doppler_move_opt[1] = step_down_opt;
        doppler_move_opt[2] = down_cycle_opt;
        doppler_move_opt[3] = wait_cycle_opt;

        return doppler_move_opt;
}

 /*
  * This function is used for register setting of power detector
  * channel_index : 0 / 1 / 2 / 3 /-1 (all channels)
  * pdt_type      : calon 0 / caloff 1 / paon 2 / paoff 3
  * if all channels on, set register of TX0
  */
void fmcw_radio_pdt_reg(fmcw_radio_t *radio, int8_t pdt_type, int32_t channel_index)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);

        /* general setting */
        RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDT_EN,0x1);
        RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDT_CALGEN_RSEL,0x1);
        fmcw_radio_switch_bank(radio, 0);
        RADIO_MOD_BANK_REG(0, TPANA2, T_MUX_2_EN, 0x1);

        /* select TPANA mux of channel */
        switch (channel_index) {
        case -1:
        case 0:
                RADIO_MOD_BANK_REG(0, TPANA2, T_MUX_2_SEL, 0x28);
                break;
        case 1:
                RADIO_MOD_BANK_REG(0, TPANA2, T_MUX_2_SEL, 0x29);
                break;
        case 2:
                RADIO_MOD_BANK_REG(0, TPANA2, T_MUX_2_SEL, 0x2A);
                break;
        case 3:
                RADIO_MOD_BANK_REG(0, TPANA2, T_MUX_2_SEL, 0x2B);
                break;
        default:
                EMBARC_PRINTF("\n\n");
                break;
        }

        /* select pdt_type */
        fmcw_radio_switch_bank(radio, 1);
        switch (pdt_type) {
        case 0:
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDT_CALGEN_EN,0x1);
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDTO_SEL,0x4);
                break;
        case 1:
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDT_CALGEN_EN,0x0);
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDTO_SEL,0x4);
                break;
        case 2:
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDT_CALGEN_EN,0x0);
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDTO_SEL,0x1);
                break;
        case 3:
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDT_CALGEN_EN,0x0);
                RADIO_MOD_BANK_CH_REG(1, channel_index, TX_TN2, PDTO_SEL,0x1);
                fmcw_radio_switch_bank(radio, 0);
                /* disable fmcw vco */
                RADIO_MOD_BANK_REG(0, FP_EN, VC_EN, 0x0);
//                 /* disable lo main ldo */
//                 RADIO_MOD_BANK_REG(0, LO_LDO0, LDO11_LMP_EN, 0x0);
//                 /* disable txlo ldo */
//                 RADIO_MOD_BANK_REG(0, LO_LDO2, LDO11_TXLO_EN, 0x0);
//                 /* disable rxlo ldo */
//                 RADIO_MOD_BANK_REG(0, LO_LDO3, LDO11_RXLO_EN, 0x0);
//                 /* disable lo main bias */
//                 RADIO_WRITE_BANK_REG(0, LO_EN0, 0x00);
//                 /* disable txlo bias */
//                 RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG3_EN, 0x0);
//                 RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG2_EN, 0x0);
//                 RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG1_EN, 0x0);
//                 RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG3_EN, 0x0);
//                 RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG2_EN, 0x0);
//                 RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG1_EN, 0x0);
// #ifdef CHIP_CASCADE
//                 /* disable lo muxio ldo */
//                 RADIO_MOD_BANK_REG(0, LO_LDO1, LDO11_LMO_EN, 0x0);
//                 /* disable lo muxio bias */
//                 RADIO_WRITE_BANK_REG(0, LO_EN2, 0x00);
// #endif
                break;
        default:
                EMBARC_PRINTF("\n\n");
                break;
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/* return rf power at certain freq of certain channel for alps series */
double fmcw_radio_rf_power_detector(fmcw_radio_t *radio, int32_t channel_index, double freq)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        baseband_t *bb = baseband_get_bb(0);
        double cal_on, cal_off, pa_on, pa_off, rf_power, cut_freq;
        int8_t part_no;

        /* lock frequency */
        fmcw_radio_single_tone(radio, freq, true);

        /* get voltage of calon, caloff, paon and paoff */
        fmcw_radio_pdt_reg(radio, 0, channel_index);
        cal_on= fmcw_radio_auxadc2_voltage(radio, 4);
        fmcw_radio_pdt_reg(radio, 1, channel_index);
        cal_off= fmcw_radio_auxadc2_voltage(radio, 4);
        fmcw_radio_pdt_reg(radio, 2, channel_index);
        pa_on= fmcw_radio_auxadc2_voltage(radio, 4);
        fmcw_radio_pdt_reg(radio, 3, channel_index);
        pa_off= fmcw_radio_auxadc2_voltage(radio, 4);
        /* after pa off, we should enable fmcwpll vco again */
        RADIO_MOD_BANK_REG(0, FP_EN, VC_EN, 0x1);
//         /* enable lo main ldo */
//         RADIO_MOD_BANK_REG(0, LO_LDO0, LDO11_LMP_EN, 0x1);
//         /* enable txlo ldo */
//         RADIO_MOD_BANK_REG(0, LO_LDO2, LDO11_TXLO_EN, 0x1);
//         /* enable rxlo ldo */
//         RADIO_MOD_BANK_REG(0, LO_LDO3, LDO11_RXLO_EN, 0x1);
//         /* enable lo main bias */
//         RADIO_WRITE_BANK_REG(0, LO_EN0, 0xFF);
//         /* enable txlo bias */
//         RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG3_EN, 0x1);
//         RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG2_EN, 0x1);
//         RADIO_MOD_BANK_REG(0, LO_EN1, LO1_TXDR_STG1_EN, 0x1);
//         RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG3_EN, 0x1);
//         RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG2_EN, 0x1);
//         RADIO_MOD_BANK_REG(0, LO_EN1, LO2_TXDR_STG1_EN, 0x1);
// #ifdef CHIP_CASCADE
//         /* enable lo muxio ldo */
//         RADIO_MOD_BANK_REG(0, LO_LDO1, LDO11_LMO_EN, 0x1);
//         if (chip_cascade_status() == CHIP_CASCADE_MASTER) {
//                 /* enable master chip lo muxio bias */
//                 RADIO_WRITE_BANK_REG(0, LO_EN2, 0x80);
//         } else {
//                 /* enable slave chip lo muxio bias */
//                 RADIO_WRITE_BANK_REG(0, LO_EN2, 0x7F);
//         }
// #endif
        fmcw_radio_single_tone(radio, freq, false);

        /* check ewlb and AIP part number, ewlb=0x45, AIP=0x42*/
        part_no = fmcw_radio_part_number(radio);
        if (part_no == 0x45) {
                cut_freq = 78;
        } else if (part_no == 0x42) {
                cut_freq = 79;
        } else {
                cut_freq = 78;
                EMBARC_PRINTF("part number not distinguished");
        }

        /* calculate according to pdt formula */
        if (freq <= cut_freq){
                rf_power = 12 * log10 ((pa_on - pa_off)/(cal_on - cal_off)) +9.5;
        }
        else{
                rf_power = 12 * log10 ((pa_on - pa_off)/(cal_on - cal_off)) +7.5;
        }
        fmcw_radio_generate_fmcw(&bb->radio);
        fmcw_radio_switch_bank(radio, old_bank);
        return rf_power;
}

void fmcw_radio_reset(void)
{
        /* This is a software reset mechanism which triggers ADC reset, bb_core reset, bb_top reset and sw_reboot */

        /* ADC reset asserted */
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_RESET_OFFSET, 4);
        raw_writel(REL_REGBASE_DMU + REG_DMU_MUX_SYNC_OFFSET, 4);
        raw_writel(REL_REGBASE_DMU + REG_DMU_ADC_RSTN_OFFSET, 0);

        /* Baseband reset asserted */
        bb_core_reset(1);
        bb_top_reset(1);

        /* Delay for ADC reset */
        UDELAY(10);

        /* ADC reset deasserted */
        raw_writel(REL_REGBASE_DMU + REG_DMU_ADC_RSTN_OFFSET, 1);

        /* Baseband reset deasserted */
        bb_top_reset(0);
        bb_core_reset(0);

       /* Radio caused software reset */
       fmcw_radio_switch_bank(NULL, 0);
       fmcw_radio_reg_write(NULL, R0_T_CBC2, 0x30 );
       MDELAY(1);
       fmcw_radio_reg_write(NULL, R0_T_CBC2, 0x00 );
}

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

void fmcw_radio_lvds_mode_norm_to_test(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        /*PMU enalbe--begin*/
        RADIO_MOD_BANK_REG(0, CBC_EN, CGM_EN, 0x1);
        RADIO_MOD_BANK_REG(0, CBC_EN, LDO_EN, 0x1);
        RADIO_MOD_BANK_REG(0, CBC_EN, BG_EN, 0x1);
        RADIO_MOD_BANK_REG(0, LDO25_PMU, EN, 0x1);
        /*PMU enalbe--end*/
        fmcw_radio_switch_bank(radio, 2);
        RADIO_WRITE_BANK_REG(2, LVDS_LDO25, 0xc0);
        RADIO_WRITE_BANK_REG(2, CH0_LVDS, 0x74);
        RADIO_WRITE_BANK_REG(2, CH1_LVDS, 0x74);
        RADIO_WRITE_BANK_REG(2, CH2_LVDS, 0x74);
        RADIO_WRITE_BANK_REG(2, CH3_LVDS, 0x74);
        RADIO_WRITE_BANK_REG(2, LVDS_CLK, 0x74);
        RADIO_WRITE_BANK_REG(2, LVDS_FRAME, 0x74);
        RADIO_WRITE_BANK_REG(2, LVDS_T0, 0xcc);
        RADIO_WRITE_BANK_REG(2, LVDS_T1, 0xcc);
        RADIO_WRITE_BANK_REG(2, LVDS_T2, 0xcc);
        fmcw_radio_switch_bank(radio, old_bank);
}

void fmcw_radio_lvds_mode_test_to_norm(fmcw_radio_t *radio)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 2);
        RADIO_WRITE_BANK_REG(2, LVDS_T0, 0x4c);
        RADIO_WRITE_BANK_REG(2, LVDS_T1, 0x4c);
        RADIO_WRITE_BANK_REG(2, LVDS_T2, 0x4c);
        fmcw_radio_switch_bank(radio, old_bank);
}


#if REFPLL_CBANK == 1
void fmcw_radio_refpll_cbank(fmcw_radio_t *radio, float temp)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        int32_t junc_temp;
        /* round temperture to 10C/step */
        junc_temp = floor(temp / 10.0) * 10;
        /* apply ldo selection when -40C < junc_temp < 120C */
        switch (junc_temp){
        case -40:
        case -30:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x0);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x7);
                break;
        case -20:
        case -10:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x4);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x6);
                break;
        case 0:
        case 10:
        case 20:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x4);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x5);
                break;
        case 30:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x4);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x4);
                break;
        case 40:
        case 50:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x4);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x3);
                break;
        case 60:
        case 70:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x7);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x3);
                break;
        case 80:
        case 90:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x7);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x2);
                break;
        case 100:
        case 110:
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x7);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x1);
                break;
        default:
                EMBARC_PRINTF("\n\n");
                break;
        }
        /* apply ldo selection when junc_temp <= -40C or junc_temp >= 120C */
        if (junc_temp <= -40) {
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x0);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x7);
        } else if (junc_temp >= 120) {
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x7);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x0);
        } else{
                EMBARC_PRINTF("\n\n");
        }

        fmcw_radio_switch_bank(radio, old_bank);
}

/* recal refpll according to junction temperature */
int32_t fmcw_radio_pll_recal(void)
{
        fmcw_radio_t *radio = NULL;
        int32_t result = E_OK;
        uint8_t old_bank;
        old_bank = fmcw_radio_switch_bank(radio, 0);

        /* Trim auxadc need 400MHz clock */
        fmcw_radio_auxadc_trim(radio);
        MDELAY(2);
        auto_lock_junc_temp = fmcw_radio_get_temperature(radio);
        if (auto_lock_junc_temp < REFPLL_CBANK_BOOT_TEMP) {
                /* Swtich clock source to 50M clock */
                /* (a) Set CLKGEN_SEL_300M (0xb20008) to 0x0. */
                raw_writel(REG_CLKGEN_SEL_300M, 0);
                /* (b) Set CLKGEN_SEL_400M (0xb2000c) to 0x0. */
                raw_writel(REG_CLKGEN_SEL_400M, 0);

                /* Set current cpu freq to 50M */
                set_current_cpu_freq(XTAL_CLOCK_FREQ);

                fmcw_radio_refpll_cbank(radio, auto_lock_junc_temp);
                /* Do auto-lock */
                result = fmcw_radio_do_refpll_cal(radio);
                if(E_OK != result){
                        fmcw_radio_switch_bank(radio, old_bank);
                        return result;
                }
                MDELAY(2);
                /* Switch to Bank 0*/
                fmcw_radio_switch_bank(radio, 0);
                /* Restore LDO value */
                RADIO_MOD_BANK_REG(0, RP_LDO1, LDO25_PLL_VSEL, 0x4);
                RADIO_MOD_BANK_REG(0, RP_LDO2, LDO11_VC_VSEL, 0x4);
                MDELAY(1);
                if(!raw_readl(REG_CLKGEN_READY_PLL)){
                        fmcw_radio_switch_bank(radio, old_bank);
                        return E_REFPLL_UNLOCK;
                }
                /* Swtich back clock source to pll */
                /* (a) Set CLKGEN_SEL_300M (0xb20008) to 0x1. */
                raw_writel(REG_CLKGEN_SEL_300M, 1);
                /* (b) Set CLKGEN_SEL_400M (0xb2000c) to 0x1. */
                raw_writel(REG_CLKGEN_SEL_400M, 1);

                /* Set current cpu freq to 300M */
                set_current_cpu_freq(PLL1_OUTPUT_CLOCK_FREQ);
        }
        fmcw_radio_switch_bank(radio, old_bank);
        return result;
}
#endif

#if (TIA_SAT == 1)
float fmcw_radio_tia_saturation_detector(fmcw_radio_t *radio, uint8_t ch)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        float tia_volt_n, tia_volt_p, tia_volt_diff;

        RADIO_MOD_BANK_REG(0, T_CBC1, MUX_DIFF_EN, 0x1);
        RADIO_MOD_BANK_CH_REG(0, ch, RX_T, OMUX_SEL, 0x8);
        RADIO_MOD_BANK_CH_REG(0, ch, RX_T, TSTMUX_SEL, 0x8);
        tia_volt_n = fmcw_radio_auxadc1_voltage(radio, 1);
        tia_volt_p = fmcw_radio_auxadc1_voltage(radio, 2);
        tia_volt_diff = fabs(tia_volt_n - tia_volt_p);
        // EMBARC_PRINTF("CH = %d, VOT_N = %.3f, VOT_P = %.3f, VOL_DIFF = %.3f\n", ch, tia_volt_n, tia_volt_p, tia_volt_diff);

        /* restore registers */
        RADIO_MOD_BANK_REG(0, T_CBC1, MUX_DIFF_EN, 0x0);
        RADIO_MOD_BANK_CH_REG(0, ch, RX_T, OMUX_SEL, 0x1);
        RADIO_MOD_BANK_CH_REG(0, ch, RX_T, TSTMUX_SEL, 0x0);

        fmcw_radio_switch_bank(radio, old_bank);
        return tia_volt_diff;
}

/* tia_step: 1 - 250ohms, 2 - 500ohms */
void fmcw_radio_set_tia_step(fmcw_radio_t *radio, uint8_t tia_step)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        for (int ch = 0; ch < MAX_NUM_RX; ch++) {
                RADIO_MOD_BANK_CH_REG(0, ch, RX_TN0, TIA_RFB_SEL, tia_step);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}

/* tia_step: 1 - 250ohms, 2 - 500ohms */
uint8_t fmcw_radio_get_tia_step(fmcw_radio_t *radio, int8_t ch)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint8_t val = (RADIO_READ_BANK_CH_REG(0, ch, RX_TN0) >> 4) & 0x0f;
        fmcw_radio_switch_bank(radio, old_bank);
        return val;
}

/* tx_pa_ldo_step: 10 - 1V, 9 - 0.968V, 8 - 0.936V */
void fmcw_radio_set_tx_pa_ldo_step(fmcw_radio_t *radio, uint8_t tx_pa_ldo_step)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        for (int ch = 0; ch < MAX_NUM_TX; ch++) {
                RADIO_MOD_BANK_CH_REG(1, ch, PA_LDO, LDO11_TX_PA_VO_SEL, tx_pa_ldo_step);
        }
        fmcw_radio_switch_bank(radio, old_bank);
}
/* tx_pa_ldo_step: 10 - 1V, 9 - 0.968V, 8 - 0.936V */
uint8_t fmcw_radio_get_tx_pa_ldo_step(fmcw_radio_t *radio, int8_t ch)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        uint8_t val = (RADIO_READ_BANK_CH_REG(1, ch, PA_LDO) >> 4) & 0x0f;
        fmcw_radio_switch_bank(radio, old_bank);
        return val;
}

/* tx_pa_bias_step: 0xAA, 0x99, 0x88, 0x77 */
void fmcw_radio_set_tx_pa_bias_step(fmcw_radio_t *radio, uint8_t tx_pa_bias_step)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        for (int ch = 0; ch < MAX_NUM_TX; ch++) {
                RADIO_WRITE_BANK_CH_REG(1, ch, TX_BI0, tx_pa_bias_step);
                RADIO_WRITE_BANK_CH_REG(1, ch, TX_BI1, tx_pa_bias_step);
                if (tx_pa_bias_step <= 0x88) {
                        RADIO_WRITE_BANK_CH_REG(1, ch, TX_BI2, tx_pa_bias_step);
                }
        }
        fmcw_radio_switch_bank(radio, old_bank);
}
/* tx_pa_bias_step: 0xAA, 0x99, 0x88, 0x77 */
uint8_t fmcw_radio_get_tx_pa_bias_step(fmcw_radio_t *radio, int8_t ch)
{
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 1);
        uint8_t val = RADIO_READ_BANK_CH_REG(1, ch, TX_BI0);
        fmcw_radio_switch_bank(radio, old_bank);
        return val;
}

int32_t search_index(uint8_t *buf,int32_t buf_len,uint8_t target_val)
{
        for (int32_t i = 0; i < buf_len; i++)
        {
                if(buf[i] == target_val){
                        return i;
                }
        }
        return -1;
}
/**
 * @name: fmcw_radio_tia_gain_and_tx_power_auto_detect
 * @msg: used for auto detect tia gain銆乼x ldo voltage and pa bias stepup
 * @param {int8_t} channel :(-1) auto slect a channel(which one tia output voltage is the max)
 *                          (0~3) specified channel
 * @param {float} threshold_H : threshold high(mV),if the channel's(specified by param "channel") voltage output is higher than threshold_H,it's will cause a auto detect
 * @param {float} threshold_L : threshold low(mV),if the channel's(specified by param "channel") voltage output is lower than threshold_H,it's will cause a auto detect
 * @return {*} E_OK if success
 */
int32_t fmcw_radio_tia_gain_and_tx_power_auto_detect(fmcw_radio_t *radio, int8_t channel, float threshold_H, float threshold_L)
{
        float V_diff_pre = 0.0;
        float V_diff_cur = 0.0;
        uint8_t tia_gain_step[TIA_GAIN_STEPS_NUM] = {0x01, 0x02};
        uint8_t tx_pa_ldo_step[TX_PA_LDO_STEPS_NUM] = {0x08, 0x09, 0x0a};
        uint8_t tx_pa_bias_step[TX_PA_BIAS_STEPS_NUM] = {0x77, 0x88, 0x99, 0xaa};
        uint8_t val = 0;
        int32_t index = 0;
        // EMBARC_PRINTF("%s ch = %d threshold_H = %.3f threshold_L = %.3f\n", __func__, channel, threshold_H, threshold_L);
        /*slect base channel and read the max V_diff*/
        if(channel == -1)
        {
                for (int i = 0; i < MAX_NUM_RX; i++)
                {
                        V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, i);
                        if(V_diff_cur > V_diff_pre)
                        {
                                channel = i;
                                V_diff_pre = V_diff_cur;
                        }
                }
                V_diff_cur = V_diff_pre;
                // EMBARC_PRINTF("auto slect ch = %d ,V_diff = %.3f\n", channel, V_diff_cur);
        }
        else if(channel >= MAX_NUM_RX)
        {
                return E_PAR;
        }
        else
        {
                V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, channel);
        }

        if(V_diff_cur > threshold_H) //if high,decrease flow
        {
                /*adjust TIA*/
                val = fmcw_radio_get_tia_step(radio, channel);
                index = search_index(tia_gain_step, TIA_GAIN_STEPS_NUM, val);
                for(int32_t i = 0; i < TIA_GAIN_STEPS_NUM; i++)
                {
                        if(index > 0)
                        {
                                index--;
                                fmcw_radio_set_tia_step(NULL, tia_gain_step[index]);
                        }
                        else
                        {
                                break;
                        }

                        V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, channel);
                        if(V_diff_cur > threshold_H)
                        {
                                continue;
                        }
                        else
                        {
                                return E_OK;
                        }
                }
                /*adjust PA LDO*/
                val = fmcw_radio_get_tx_pa_ldo_step(radio, channel);
                index = search_index(tx_pa_ldo_step, TX_PA_LDO_STEPS_NUM, val);
                for(int32_t i = 0; i < TX_PA_LDO_STEPS_NUM; i++)
                {
                        if(index > 0)
                        {
                                index--;
                                fmcw_radio_set_tx_pa_ldo_step(NULL, tx_pa_ldo_step[index]);
                        }
                        else
                        {
                                break;
                        }

                        V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, channel);
                        if(V_diff_cur > threshold_H)
                        {
                                continue;
                        }
                        else
                        {
                                return E_OK;
                        }
                }
                /*adjust PA BI*/
                val = fmcw_radio_get_tx_pa_bias_step(radio, channel);
                index = search_index(tx_pa_bias_step, TX_PA_BIAS_STEPS_NUM, val);
                for(int32_t i = 0; i < TX_PA_BIAS_STEPS_NUM; i++)
                {
                        if(index > 0)
                        {
                                index--;
                                fmcw_radio_set_tx_pa_bias_step(NULL, tx_pa_bias_step[index]);
                        }
                        else
                        {
                                break;
                        }

                        V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, channel);
                        if(V_diff_cur > threshold_H)
                        {
                                continue;
                        }
                        else
                        {
                                return E_OK;
                        }
                }
                return E_SYS;
        }
        else if(V_diff_cur < threshold_L) //if low,increase flow
        {
                /*adjust PA BI*/
                val = fmcw_radio_get_tx_pa_bias_step(radio, channel);
                index = search_index(tx_pa_bias_step, TX_PA_BIAS_STEPS_NUM, val);
                for(int32_t i = 0; i < TX_PA_BIAS_STEPS_NUM; i++)
                {
                        if((index >= 0) && (index < TX_PA_BIAS_STEPS_NUM - 1))
                        {
                                index++;
                                fmcw_radio_set_tx_pa_bias_step(NULL, tx_pa_bias_step[index]);
                        }
                        else
                        {
                                break;
                        }

                        V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, channel);
                        if(V_diff_cur < threshold_L)
                        {
                                continue;
                        }
                        else
                        {
                                return E_OK;
                        }
                }
                /*adjust PA LDO*/
                val = fmcw_radio_get_tx_pa_ldo_step(radio, channel);
                index = search_index(tx_pa_ldo_step, TX_PA_LDO_STEPS_NUM, val);
                for(int32_t i = 0; i < TX_PA_LDO_STEPS_NUM; i++)
                {
                        if((index >= 0) && (index < TX_PA_LDO_STEPS_NUM - 1))
                        {
                                index++;
                                fmcw_radio_set_tx_pa_ldo_step(NULL, tx_pa_ldo_step[index]);
                        }
                        else
                        {
                                break;
                        }

                        V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, channel);
                        if(V_diff_cur > threshold_L)
                        {
                                continue;
                        }
                        else
                        {
                                return E_OK;
                        }
                }
                /*adjust TIA*/
                val = fmcw_radio_get_tia_step(radio, channel);
                index = search_index(tia_gain_step, TIA_GAIN_STEPS_NUM, val);
                for(int32_t i = 0; i < TIA_GAIN_STEPS_NUM; i++)
                {
                        if((index >= 0) && (index < TIA_GAIN_STEPS_NUM - 1))
                        {
                                index++;
                                fmcw_radio_set_tia_step(NULL, tia_gain_step[index]);
                        }
                        else
                        {
                                break;
                        }

                        V_diff_cur = fmcw_radio_tia_saturation_detector(NULL, channel);
                        if(V_diff_cur < threshold_L)
                        {
                                continue;
                        }
                        else
                        {
                                return E_OK;
                        }
                }
                return E_SYS;
        }
        else //no need adjust
        {
                return E_OK;
        }

}

#endif

void fmcw_radio_set_reg(fmcw_radio_t *radio)
{
    uint8_t oldBank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
    fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_0, REG_L(radio->start_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_1, REG_M(radio->start_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_2, REG_H(radio->start_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_START_FREQ_1_3, REG_INT(radio->start_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_0, REG_L(radio->stop_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_1, REG_M(radio->stop_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_2, REG_H(radio->stop_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_STOP_FREQ_1_3, REG_INT(radio->stop_freq));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_0, REG_L(radio->step_up));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_1, REG_M(radio->step_up));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_2, REG_H(radio->step_up));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_UP_FREQ_1_3, REG_INT(radio->step_up));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_0, REG_L(radio->step_down));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_1, REG_M(radio->step_down));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_2, REG_H(radio->step_down));
    fmcw_radio_reg_write(radio, R5_FMCW_STEP_DN_FREQ_1_3, REG_INT(radio->step_down));
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_0, REG_L(radio->cnt_wait));
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_1, REG_M(radio->cnt_wait));
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_2, REG_H(radio->cnt_wait));
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_3, REG_INT(radio->cnt_wait));
    fmcw_radio_switch_bank(radio, oldBank);
}

void fmcw_set_chirp_period(fmcw_radio_t *radio)
{
    uint8_t oldBank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_0, REG_L(radio->cnt_wait));
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_1, REG_M(radio->cnt_wait));
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_2, REG_H(radio->cnt_wait));
    fmcw_radio_reg_write(radio, R5_FMCW_IDLE_1_3, REG_INT(radio->cnt_wait));
    fmcw_radio_switch_bank(radio, oldBank);
}

