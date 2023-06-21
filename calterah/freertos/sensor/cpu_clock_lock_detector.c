#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"
#include "alps_emu_reg.h"
#include "radio_ctrl.h"
#include "cpu_clock_lock_detector.h"

#define DISABLE_REBOOT_LIMIT    1


/****************************************************
 * NAME         : cpu_clock_lock_detector_init
 * DESCRIPTIONS : sm5, cpu clock lock detector init
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void cpu_clock_lock_detector_init(void)
{
        uint32_t value;
#if DISABLE_REBOOT_LIMIT == 1
        // disble reboot limit
        value = raw_readl(REG_EMU_REBOOT_LIMIT);
        raw_writel(REG_EMU_REBOOT_LIMIT, value | 1 << 2);
#endif
        //set EMU.RF_ERR_SS1_MARK.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_RF_ERR_SS1_MASK);
        raw_writel(REG_EMU_RF_ERR_SS1_MASK, value | (1 << 4));
        //set EMU.RF_ERR_SS1_ENA.bit_shift bit to 0x1
        value = raw_readl(REG_EMU_RF_ERR_SS1_ENA);
        raw_writel(REG_EMU_RF_ERR_SS1_ENA, value | (1 << 4));
        //configure radio
        fmcw_radio_reg_write(NULL, 0, 0);
        fmcw_radio_reg_write(NULL, 125, 0x15);
        fmcw_radio_reg_write(NULL, 0, 0xa);
        fmcw_radio_reg_write(NULL, 110, 0x2);
        fmcw_radio_reg_write(NULL, 0, 0);
}

/****************************************************
 * NAME         : cpu_clock_lock_detector_fault_injection
 * DESCRIPTIONS : sm5 fault injection
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void cpu_clock_lock_detector_fault_injection(void)
{
        fmcw_radio_reg_write(NULL, 0, 0);
        fmcw_radio_reg_write(NULL, 125, 0x14);
}

/****************************************************
 * NAME         : check_cpu_clock_lock_detector_error_code
 * DESCRIPTIONS : read rf error code
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void check_cpu_clock_lock_detector_error_code(void)
{
        uint32_t value;

        //check emu reboot cnt
        EMBARC_PRINTF("\r\n");
        value = raw_readl(REG_EMU_REBOOT_CNT);
        EMBARC_PRINTF("EMU_REBOOT_CNT_READ: 0x%x, value: %d.\n", REG_EMU_REBOOT_CNT, value);
        EMBARC_PRINTF("\r\n");
        //check emu rf SS1 error code
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE0);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE0: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_SS1_CODE0, value);
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE1);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE1: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_SS1_CODE1, value);
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE2);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE2: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_SS1_CODE2, value);
        value = raw_readl(REG_EMU_RF_ERR_SS1_CODE3);
        EMBARC_PRINTF("EMU_RF_ERR_SS1_CODE3: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_SS1_CODE3, value);
        EMBARC_PRINTF("\r\n");
}
