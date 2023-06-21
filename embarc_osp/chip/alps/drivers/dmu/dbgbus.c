#include "embARC_toolchain.h"
#include "embARC_error.h"

#include "mux.h"
#include "embARC.h"
#include "embARC_debug.h"

#include "alps_module_list.h"
#include "alps_dmu_reg.h"
#include "alps_dmu.h"
#include "baseband_hw.h"
#include "gpio_hal.h"
#include "dw_gpio.h"
#include "dbgbus.h"



#if (defined(CHIP_CASCADE) || (USE_50PIN_DCK_BOARD == 1))
/* The cascade data acquisition support board is as follows: */
/* CAL0074, CAL0100, CAL0107, CAL0123 */
/* In CASCADE mode, GPIO 16/17/18/19/20 is used as SPI communication between master and slave chip */
#define DATA_DUMP_RESET 22
#else
#define DATA_DUMP_DONE 17
#define DATA_DUMP_RESET 18
#define HIL_INPUT_READY 19
#endif

/* dbgbus raw operating methods. */
void dbgbus_input_config(void)
{
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_VAL_OEN_OFFSET, DBGBUS_OUTPUT_DISABLE);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_DISABLE);
        dbgbus_enable(1); /* enable dbgbus clock */
}

void dbgbus_dump_enable(uint8_t dbg_src)
{
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_SRC_OFFSET, dbg_src);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_VAL_OEN_OFFSET, DBGBUS_OUTPUT_ENABLE);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_ENABLE);
}

void dump_reset(void)
{
        uint32_t dmu_sel_mode = sys_dmu_mode_get();
        if (dmu_sel_mode == SYS_DMU_SEL_DBG) {
                dbgbus_dump_reset();              /* reset fpga board */
        } else {
                int32_t ret = gpio_set_direct(DATA_DUMP_RESET, DW_GPIO_DIR_OUTPUT);
                if (ret != 0) {
                        EMBARC_PRINTF("Dir gpio(%d) error! ret %d \n", DATA_DUMP_RESET, ret);
                }
                gpio_write(DATA_DUMP_RESET, 0);
                chip_hw_udelay(10);
                gpio_write(DATA_DUMP_RESET, 1);
                chip_hw_udelay(10);
                gpio_write(DATA_DUMP_RESET, 0);
        }
}

void dump_done(void)
{
#if (!defined(CHIP_CASCADE) && (USE_50PIN_DCK_BOARD == 0))
        uint32_t dmu_sel_mode = sys_dmu_mode_get();
        if (dmu_sel_mode == SYS_DMU_SEL_DBG) {
                dbgbus_dump_done();               /* reset fpga board */
        } else {
                int32_t ret = gpio_set_direct(DATA_DUMP_DONE, DW_GPIO_DIR_OUTPUT);
                if (ret != 0) {
                        EMBARC_PRINTF("Dir gpio(%d) error! ret %d \n", DATA_DUMP_DONE, ret);
                }
                gpio_write(DATA_DUMP_DONE, 0);
                chip_hw_udelay(10);
                gpio_write(DATA_DUMP_DONE, 1);
                chip_hw_udelay(10);
                gpio_write(DATA_DUMP_DONE, 0);
        }
#endif
}

/* gpio_23/gpio_19, ready signal of hil */
/* ready signal should be sent to FPGA at the beginning of every hil frame */
/* HIL is not supported on cascade board */
void dbgbus_hil_ready(void)
{
#ifndef CHIP_CASCADE
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_SRC_OFFSET, DBG_SRC_CPU);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_19_OEN);
        /* Only 1 posedge is needed */
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_RESET);   /* write 0 */

        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_19_MASK); /* write 1 */
#endif
}

/* gpio_22/gpio_18, reset signal */
/* reset signal should be sent to FPGA at the beginning of every data collection frame */
/* reset signal and done signal should be used in pair */
void dbgbus_dump_reset(void)
{
#ifdef CHIP_CASCADE
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_SRC_OFFSET, DBG_SRC_CPU);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_22_OEN);
        /* Only 1 posedge is needed */
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_RESET);   /* write 0 */

        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_22_MASK); /* write 1 */
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_DISABLE);
#else
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_SRC_OFFSET, DBG_SRC_CPU);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_18_OEN);
        /* Only 1 posedge is needed */
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_RESET);   /* write 0 */

        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_18_MASK); /* write 1 */
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_DISABLE);
#endif
}

/* gpio_21/gpio_17, done signal */
/* done signal should be sent to FPGA at the end of every data collection frame */
/* reset signal and done signal should be used in pair */
/* Done signal is not required on the cascade board */
void dbgbus_dump_done(void)
{
#ifndef CHIP_CASCADE
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_SRC_OFFSET, DBG_SRC_CPU);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_17_OEN);
        /* Only 1 posedge is needed */
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_RESET);   /* write 0 */

        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_17_MASK); /* write 1 */
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_DISABLE);

#endif
}

void dbgbus_dump_disable(void)
{
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_SRC_OFFSET, DBG_SRC_CPU);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_VAL_OEN_OFFSET, DBGBUS_OUTPUT_DISABLE);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_DISABLE);
}

void dbgbus_free_run_enable(void)
{
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_SRC_OFFSET, DBG_SRC_CPU);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_VAL_OEN_OFFSET, DBGBUS_OUTPUT_DISABLE);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DAT_OEN_OFFSET, DBGBUS_DAT_20_OEN);
}

void gpio_free_run_sync(void)
{
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_20_MASK);
        raw_writel(REL_REGBASE_DMU + REG_DMU_DBG_DOUT_OFFSET, DBGBUS_DAT_RESET);
}

void dbgbus_dump_start( uint8_t dbg_src)
{
        /* gpio config */
        io_mux_dbgbus_dump();
        dbgbus_dump_reset(); /* reset fpga board */
        dbgbus_dump_enable(dbg_src);

        dbgbus_enable(1); /* enable dbgbus clock, !! this should be at the last line of this function !! */
}

void dbgbus_dump_stop(void)
{
        dump_done();
        io_mux_dbgbus_mode_stop();
}

#ifdef CHIP_CASCADE
// Note, debugbus(gpio) 5, 6, 12 are multiplexed with cascade control signals
// cascade_irq -- gpio_dat_5, 6, fmcw_start -- gpio_dat_12
// when  data dump, these 3 pins should be switched to dbgbus
// after data dump, these 3 pins should be switched to cascade control immediately.

void dbgbus_dump_cascade_switch(bool enable)
{
        if (enable) {
                io_mux_casade_irq_disable();
                io_mux_fmcw_start_sel_disable();
                dbgbus_enable(1); /* enable dbgbus clock */
        } else {
                dbgbus_enable(0); /* disable dbgbus clock firstly to avoid abnormal signal */
                io_mux_casade_irq();
                io_mux_fmcw_start_sel();
        }
}
#endif

