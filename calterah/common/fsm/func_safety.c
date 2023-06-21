#include "embARC.h"
#include "embARC_debug.h"
#include "alps_emu_reg.h"
#include "func_safety.h"
#include "i2c_hal.h"
#include "stdbool.h"
#include "string.h"
#include "dbg_gpio_reg.h"
#include "alps_dmu_reg.h"
#include "alps_mmap.h"
#include "alps_interrupt.h"
#include "gpio_hal.h"
#include "dw_gpio.h"
#include "alps_clock.h"
#include "baseband_alps_FM_reg.h"
#include "alps_mp_radio_reg.h"
#include "calterah_error.h"
#include "baseband.h"
#include "cmd_reg.h"
#include "radio_reg.h"
#include "radio_ctrl.h"
#include "cascade.h"
#include <stdlib.h>
#include <math.h>
#include "sensor_config.h"
#include "uart_hal.h"
#include "baseband_cli.h"
#include "can_reg.h"
#include "dw_i2c_reg.h"
#include "dw_i2c.h"
#include "crc_hal.h"
#include "spi_hal.h"
#include "dw_ssi.h"
#include "can_signal_interface.h"
#include "can_hal.h"
#include "dw_ssi_reg.h"
#include "arc_wdg.h"
#include "baseband_dpc.h"
#include "track_cli.h"
#include "FreeRTOS_CLI.h"
#include "alps_timer.h"
#include "fmcw_radio.h"

#define fusa_delay_ms   chip_hw_mdelay
#define fusa_delay_us   chip_hw_udelay
#define core_reg_read       _arc_aux_read
#define core_reg_write      _arc_aux_write
#define raw_read_op         raw_readl
#define raw_write_op        raw_writel
#define FUSA_CONF_MAX (sizeof (fusaConfList) / sizeof (fusa_config_t))

static uint8_t CRC8_ROHC(uint8_t array[], uint8_t length) ;
static void emu_rf_error_ss1_cpu_20_isr(void *params);
static void emu_digital_error_ss1_cpu_22_isr(void *params);
static void func_safety_set_dmu_irq_0_32_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss1_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss1_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss1_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss1_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss2_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_rf_err_ss2_ena(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss2_mask(uint8_t bit_shift);
static void func_safety_set_reg_emu_dig_err_ss2_ena(uint8_t bit_shift);
static void func_safety_clear_reg_emu_rf_err_irq_ena(uint8_t bit_shift);
static void func_safety_sm_spi_loopback(uint8_t func_safety_error_type);
static void fusa_on_time_error_pin_check(void);
static bool func_safety_sm_vga_consistence_check(void);
static void func_safety_sm_periodic_readback_configuration_registers(uint8_t func_safety_error_type);


/* Parameter */
bool bb_frame_start_flag = false;
bool ldo_set_part_flag = false;
volatile uint8_t sm1_ldo_part_cnt = 0x00;
bool periodic_sm_finish_flag = false;
bool sample_adc_running_flag = false;
uint32_t fusa_power_on_check_status[12U];  //stored power on check status of safety module

#if (FUNC_SAFETY_CLI == FEATURE_ON)
static BaseType_t fusa_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
void func_safety_err_inject_handler(int32_t err_item, int32_t param);
#endif
safety_cycle_t FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_1_CYCLE;
static uint8_t can_send_status = CAN_SEND_STATUS_IDLE;

#if (FUNC_SAFETY_CLI == FEATURE_ON)
/* fusa command */
static const CLI_Command_Definition_t fusa_command = {
        "fusa",
        "fusa \n\r"
        "\tStart reserve \n"
        "\err_inject <fusa_item> <err_type>\n",
        fusa_command_handler,
        -1
};
/* helpers */
static void print_help(char* buffer, size_t buff_len, const CLI_Command_Definition_t* cmd)
{
        uint32_t tot_count = 0;
        int32_t count = sprintf(buffer, "Wrong input\n\r %s", cmd->pcHelpString);
        EMBARC_ASSERT(count > 0);
        tot_count += count;
        EMBARC_ASSERT(tot_count < buff_len);
}
static BaseType_t fusa_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
        const char *param1, *param2, *param3;
        BaseType_t len1, len2, len3;
        param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
        param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
        param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);

        int32_t count = 0;
        int32_t count1 = 0;

        /* get parameter 1, 2*/
        if (param1 != NULL) {
                if (strncmp(param1, "err_inject", 10) == 0)
                {
                        if (param2 == NULL){
                                count = -1;
                        }
                        else
                        {
                                count = strtol(param2, NULL, 0);
                                count1 = strtol(param3, NULL, 0);
                        }
                        if(count >= 0){
                                // log_fusa("[fusa] err inject: %d\r\n", count);
                                func_safety_err_inject_handler(count,count1);
                        }
                        else{
                                log_fusa("err num error!\r\n");
                        }
                }
                else
                {
                        print_help(pcWriteBuffer, xWriteBufferLen, &fusa_command);
                        return pdFALSE;
                }
        }
        return pdFALSE;
}
#endif

/*support function-safety configure list */
static const fusa_config_t fusaConfList[] =
{
        /* index                  sm number                error type        open flag  */
        {SM_INDEX_0,        FUNC_SAFETY_ITEM_SM1,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM1},
        {SM_INDEX_1,        FUNC_SAFETY_ITEM_SM2,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM2},
        {SM_INDEX_2,        FUNC_SAFETY_ITEM_SM3,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM3},
        {SM_INDEX_3,        FUNC_SAFETY_ITEM_SM4,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM4},
        {SM_INDEX_4,        FUNC_SAFETY_ITEM_SM5,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM5},
        {SM_INDEX_5,        FUNC_SAFETY_ITEM_SM6,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM6},
        {SM_INDEX_6,        FUNC_SAFETY_ITEM_SM8,        SAFE_STATE_IRQ,        SAFETY_FEATURE_SM8},
        {SM_INDEX_7,        FUNC_SAFETY_ITEM_SM11,       SAFE_STATE_IRQ,        SAFETY_FEATURE_SM11},
        {SM_INDEX_8,        FUNC_SAFETY_ITEM_SM12,       SAFE_STATE_IRQ,        SAFETY_FEATURE_SM12},
        {SM_INDEX_9,        FUNC_SAFETY_ITEM_SM13,       SAFE_STATE_IRQ,        SAFETY_FEATURE_SM13},
        {SM_INDEX_10,        FUNC_SAFETY_ITEM_SM14,       SAFE_STATE_IRQ,       SAFETY_FEATURE_SM14},
        {SM_INDEX_11,        FUNC_SAFETY_ITEM_SM201,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM201},
        {SM_INDEX_12,        FUNC_SAFETY_ITEM_SM101,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM101},
        {SM_INDEX_13,        FUNC_SAFETY_ITEM_SM102,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM102},
        {SM_INDEX_14,        FUNC_SAFETY_ITEM_SM103,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM103},
        {SM_INDEX_15,        FUNC_SAFETY_ITEM_SM104,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM104},
        {SM_INDEX_16,        FUNC_SAFETY_ITEM_SM105,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM105},
        {SM_INDEX_17,        FUNC_SAFETY_ITEM_SM106,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM106},
        {SM_INDEX_18,        FUNC_SAFETY_ITEM_SM107,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM107},
        {SM_INDEX_19,        FUNC_SAFETY_ITEM_SM108,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM108},
        {SM_INDEX_20,        FUNC_SAFETY_ITEM_SM109,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM109},
        {SM_INDEX_21,        FUNC_SAFETY_ITEM_SM120,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM120},
        {SM_INDEX_22,        FUNC_SAFETY_ITEM_SM121,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM121},
        {SM_INDEX_23,        FUNC_SAFETY_ITEM_SM122,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM122},
        {SM_INDEX_24,        FUNC_SAFETY_ITEM_SM123,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM123},
        {SM_INDEX_25,        FUNC_SAFETY_ITEM_SM124,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM124},
        {SM_INDEX_26,        FUNC_SAFETY_ITEM_SM125,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM125},
        {SM_INDEX_27,        FUNC_SAFETY_ITEM_SM126,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM126},
        {SM_INDEX_28,        FUNC_SAFETY_ITEM_SM129,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM129},
        {SM_INDEX_29,        FUNC_SAFETY_ITEM_SM130,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM130},
        {SM_INDEX_30,        FUNC_SAFETY_ITEM_SM133,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM133},
        {SM_INDEX_31,        FUNC_SAFETY_ITEM_SM202,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM202},
        {SM_INDEX_32,        FUNC_SAFETY_ITEM_SM206,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM206},
        {SM_INDEX_33,        FUNC_SAFETY_ITEM_SM805,      SAFE_STATE_IRQ,       SAFETY_FEATURE_SM805}
};

void fusa_disable_all(void)
{
        raw_write_op(REG_EMU_RF_ERR_IRQ_ENA, 0);
        raw_write_op(REG_EMU_RF_ERR_SS1_ENA, 0);
        raw_write_op(REG_EMU_RF_ERR_SS2_ENA, 0);
        raw_write_op(REG_EMU_DIG_ERR_IRQ_ENA, 0);
        raw_write_op(REG_EMU_DIG_ERR_SS1_ENA, 0);
        raw_write_op(REG_EMU_DIG_ERR_SS2_ENA, 0);
}

float fusa_get_current_time_ms(void)
{
        return (float)(((float)core_reg_read(AUX_RTC_LOW))/(get_current_cpu_freq()/1000));
}

void core_reg_bit_set(uint32_t reg,uint32_t bit)
{
    uint32_t temp = core_reg_read(reg);
    temp |= 0x00000001 << bit;
    core_reg_write(reg,temp);
}

void core_reg_bit_clr(uint32_t reg,uint32_t bit)
{
    uint32_t temp = core_reg_read(reg);
    temp &= ~(0x00000001 << bit);
    core_reg_write(reg,temp);
}

/****************************************************
 * NAME         : led_d1_init
 * DESCRIPTIONS : led d1 pin init, used to detect FDTI
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void led_d1_init(void) {
        int32_t ret = E_OK;

        ret = gpio_set_direct(LED_D1_NO, DW_GPIO_DIR_OUTPUT);
        if (ret != 0) {
                log_fusa("Dir gpio(%d) error! ret %d \n", LED_D1_NO, ret);
        }
}

/****************************************************
 * NAME         : clear_mem_mac_by_sw
 * DESCRIPTIONS : clear mem_mac
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void clear_mem_mac_by_sw(void)
{
        baesband_frame_interleave_cnt_clr();
        baseband_t* bb = baseband_get_rtl_frame_type();
        baseband_hw_t *bb_hw = &bb->bb_hw;

        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_MAC); //Store bank selected
        uint32_t *mem_mac_offset = 0; // offset = 0
        for (uint16_t loop= 0; loop< 16384; loop++) { //4x4096 loop
                 baseband_write_mem_table(bb_hw, *mem_mac_offset, 0);
                 (*mem_mac_offset)++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank
}

/****************************************************
 * NAME         : clear_all_bb_memory
 * DESCRIPTIONS : clear all bb memory
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void clear_all_bb_memory(baseband_hw_t *bb_hw)
{
        uint32_t mem_offset;
        uint32_t i;
        uint32_t old;
        uint32_t value;
        /*baesband_frame_interleave_cnt_clr();
        baseband_t* bb = baseband_get_cur_bb();
        baseband_hw_t *bb_hw = &bb->bb_hw;*/

        //clear BB MEM_WIN
        /*old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_WIN); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 8192; i++) { //32K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_NVE
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_NVE); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 4096; i++) { //16K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_BUF
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 524288; i++) { //2M
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_COE
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_COE); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 16384; i++) { //64K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank*/

        //clear BB MEM_MAC
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_MAC); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 16384; i++) { //64K
             baseband_write_mem_table(bb_hw, mem_offset, 0);
             mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_RLT_DOA
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_RLT); //Store bank selected
        memset((void *)BB_MEM_BASEADDR, 0, 1024 * sizeof(obj_info_t));
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_ANC
        /*old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_ANC); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 131072; i++) { //512K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank

        //clear BB MEM_DML
        old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_DML); //Store bank selected
        mem_offset = 0;
        for (i = 0; i < 8192; i++) { //32K
                 baseband_write_mem_table(bb_hw, mem_offset, 0);
                 mem_offset++;
        }
        baseband_switch_mem_access(bb_hw, old); // Restore back to old bank*/

        //clear BB MEM_WIN
        if (bb_hw->frame_type_id == 0) { // run one time
                //func_safety_enable_bb_mem_ecc();
                value = raw_read_op(BB_REG_SYS_ECC_ENA);
                raw_write_op(BB_REG_SYS_ECC_ENA, value | (1 << 1));
                value = raw_read_op(BB_REG_SYS_ECC_ENA);

                old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_WIN); //Store bank selected
                mem_offset = 0;
                for (i = 0; i < 8192; i++) { //32K
                         baseband_write_mem_table(bb_hw, mem_offset, 0);
                         mem_offset++;
                }
                baseband_switch_mem_access(bb_hw, old); // Restore back to old bank
        }
}

/****************************************************
 * NAME         : emu_rf_error_irq_cpu_19_isr
 * DESCRIPTIONS : emu rf error irq cpu 19 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_rf_error_irq_cpu_19_isr(void *params) {
        uint32_t value;
        uint32_t value_clear;

        int_disable(INT_RF_ERROR_IRQ);

        portENTER_CRITICAL();
        value = raw_read_op(REG_EMU_RF_ERR_IRQ_STA);
        portEXIT_CRITICAL();

        log_fusa("[fusa_isr] REG_EMU_RF_ERR_IRQ_STA = 0x%x  time = %f\n",value,fusa_get_current_time_ms());

        if (value) {
                if (value & (1 << 0)) {        //LDO_MT bit
                        value_clear = fmcw_radio_switch_bank(NULL, 10);
                        log_fusa("LDO_MT bit irq set. status 0x47~0x4b = 0x%02x %02x %02x %02x %02x\n",
                                fmcw_radio_reg_read(NULL,0x47),
                                fmcw_radio_reg_read(NULL,0x48),
                                fmcw_radio_reg_read(NULL,0x49),
                                fmcw_radio_reg_read(NULL,0x4a),
                                fmcw_radio_reg_read(NULL,0x4b));
                        fmcw_radio_switch_bank(NULL, value_clear);
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                        func_safety_clear_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_0_SHIFT);
                } else if (value & (1 << 1)) {        //AVDD33_MT bit
                        log_fusa("AVDD33_MT bit irq set.\n");
                        // fmcw_radio_sm_avdd33_monitor_setting(NULL);
                        /* restart monitor */
                        fmcw_radio_sm_avdd33_monitor_IRQ(NULL, false);
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 2)) {        //DVDD11_MT bit
                        log_fusa("DVDD11_MT bit irq set.\n");
                        /* restart monitor */
                        fmcw_radio_sm_dvdd11_monitor_IRQ(NULL, false);
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 3)) {        //VBG_MT bit
                        log_fusa("VBG_MT bit irq set.\n");
                        /* restart monitor */
                        fmcw_radio_sm_bg_monitor_IRQ(NULL, false);
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 4)) {        //PLL_LOCK_DT bit
                        log_fusa("PLL_LOCK_DT bit irq set.\n");
                        /* restart monitor */
                        // fmcw_radio_reg_write(NULL, 0, 0);
                        // fmcw_radio_reg_write(NULL, 0x7d, 0x05);
                        // fmcw_radio_reg_write(NULL, 0x7d, 0x15);
                        // fmcw_radio_reg_write(NULL, 0, 0xa);
                        // fmcw_radio_reg_write(NULL, 0x6e, 0x0);
                        // fmcw_radio_reg_write(NULL, 0x6e, 0x2);
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 5)) {        //RF_POWER_DT bit
                        log_fusa("RF_POWER_DT bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                        func_safety_clear_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_5_SHIFT);
                // } else if (value & (1 << 6)) {        //TX_BALL_DT bit //zhonglei:unrealized
                //         log_fusa("TX_BALL_DT bit irq set.\n");
                //         value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                //         raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 7)) {        //RX_SAT_DT bit
                        //log_fusa("RX_SAT_DT bit irq set.\n");
                        log_fusa("RX_SAT_DT bit irq set. status = 0x%x\n",BB_READ_REG(NULL, AGC_IRQ_STATUS));
                        BB_WRITE_REG(NULL, AGC_SAT_CNT_CLR_FRA,  1);
                        BB_WRITE_REG(NULL, AGC_IRQ_CLR, 0xFFF);
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                        // func_safety_clear_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                } else if (value & (1 << 8)) {                        //CHIRP_MT bit
                        log_fusa("CHIRP_MT bit set irq set.\n");
                        value_clear = fmcw_radio_switch_bank(NULL, 10);
                        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x1);
                        RADIO_MOD_BANK_REG(10, ITF_IRQ_CTRL_4, ITF_FM_CLEAR, 0x0);
                        fmcw_radio_switch_bank(NULL, value_clear);
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 9)) {                        //TEMP_OVER bit
                        //gpio_write(LED_D1_NO, LED_D1_OFF);//end test
                        //set EMU.SS1_CTRL to 0x1
                        log_fusa("TEMP_OVER bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                        // func_safety_clear_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //value = raw_read_op(REG_EMU_SS1_CTRL);
                        //log_fusa("set EMU.SS1_CTRL to 0x1_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_SS1_CTRL, value);
                } else if (value & (1 << 10)) {                        //IF_LOOPBACK bit
                        log_fusa("IF_LOOPBACK bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 11)) {                        //RX_LOOPBACK bit
                        log_fusa("RF_LOOPBACK bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else if (value & (1 << 12)) { //GAIN_CK VGA gain consistence check bit
                        log_fusa("GAIN_CK bit set irq set.\n");
                        value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
                        raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
                } else {
                        log_fusa("Unused bit irq set.\n");
                }
        }

        int_enable(INT_RF_ERROR_IRQ);
}

/****************************************************
 * NAME         : emu_rf_error_ss1_cpu_20_isr
 * DESCRIPTIONS : emu rf error ss1 cpu 20 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_rf_error_ss1_cpu_20_isr(void *params) {
        uint32_t value;
        //uint32_t value_clear;

        //disable rf error irq
        int_disable(INT_RF_ERROR_SS1);

        portENTER_CRITICAL();
        log_fusa("emu rf error ss1 cpu 20 isr.\n");
        // check EMU.RF_ERR_SS1_STA bit
        value = raw_read_op(REG_EMU_RF_ERR_SS1_STA);
        log_fusa("check EMU.RF_ERR_SS1_STA_ADDR: 0x%x, value: 0x%x.\n",
                        REG_EMU_RF_ERR_SS1_STA, value);
        //clear the interrupt flag that lead to 20 isr
        //value_clear = raw_read_op(REG_EMU_RF_ERR_CLR);
        //log_fusa("check EMU.REG_EMU_RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value_clear);
        //raw_write_op(REG_EMU_RF_ERR_CLR, value | value_clear);
        //chip_hw_mdelay(2000);
        portEXIT_CRITICAL();
        if (value) {
                if (value & (1 << 0)) {        //LDO_MT bit
                        log_fusa("LDO_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 1)) {        //AVDD33_MT bit
                        log_fusa("AVDD33_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 2)) {        //DVDD11_MT bit
                        log_fusa("DVDD11_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 3)) {        //VBG_MT bit
                        log_fusa("VBG_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 4)) {        //PLL_LOCK_DT bit
                        log_fusa("PLL_LOCK_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 5)) {        //RF_POWER_DT bit
                        log_fusa("RF_POWER_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 6)) {        //TX_BALL_DT bit
                        log_fusa("TX_BALL_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 7)) {        //RX_SAT_DT bit
                        //clear EMU RF saturate detect irq
                        //value = raw_read_op(REG_EMU_RF_ERR_CLR);
                        //raw_write_op(REG_EMU_RF_ERR_CLR, value | (1 << 7));
                        log_fusa("RX_SAT_DT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 10)) {                        //IF_LOOPBACK bit
                        log_fusa("IF_LOOPBACK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 11)) {                        //RX_LOOPBACK bit
                        log_fusa("RX_LOOPBACK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 12)) { //GAIN_CK VGA gain consistence check bit
                        log_fusa("GAIN_CK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 8)) {                        //CHIRP_MT bit
                        log_fusa("CHIRP_MT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 9)) {                        //TEMP_OVER bit
                        //gpio_write(LED_D1_NO, LED_D1_OFF);//end test
                        //set EMU.SS1_CTRL to 0x1
                        log_fusa("TEMP_OVER bit set EMU.SS1_CTRL to 0x1.\n");
                        value = 0x1;
                        raw_write_op(REG_EMU_SS1_CTRL, value);
                        //value = raw_read_op(REG_EMU_SS1_CTRL);
                        //log_fusa("set EMU.SS1_CTRL to 0x1_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_SS1_CTRL, value);
                } else {
                        //enable rf error irq
                        int_enable(INT_RF_ERROR_SS1);
                }
        } else {
                //enable rf error irq
                int_enable(INT_RF_ERROR_SS1);
        }
}

/****************************************************
 * NAME         : emu_digital_error_irq_cpu_21_isr
 * DESCRIPTIONS : emu digital error irq cpu 21 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_digital_error_irq_cpu_21_isr(void *params) {
        uint32_t value;
        uint32_t value_clear;

        //disable digital error irq
        int_disable(INT_DG_ERROR_IRQ);

        portENTER_CRITICAL();
        value = raw_read_op(REG_EMU_DIG_ERR_IRQ_STA);
        portEXIT_CRITICAL();

        log_fusa("[fusa_isr] REG_EMU_DIG_ERR_IRQ_STA = 0x%x  time = %f\n",value,fusa_get_current_time_ms());
        if (value) {
                if (value & (1 << 0)) {
                        log_fusa("BB_LBIST bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 1)) {
                        log_fusa("CPU_TCM_ECC bit irq set.\n");
                        core_reg_bit_set(REG_CPU_ERP_CTRL,31);
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 2)) {
                        log_fusa("CPU_CACHE_ECC bit irq set.\n");
                        core_reg_bit_set(REG_CPU_ERP_CTRL,31);
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 3)) {
                        log_fusa("BB_SRAM_ECC bit irq set.\n");
                        raw_write_op(BB_REG_SYS_ECC_SB_CLR, raw_read_op(BB_REG_SYS_ECC_SB_STATUS));
                        raw_write_op(BB_REG_SYS_ECC_DB_CLR, raw_read_op(BB_REG_SYS_ECC_DB_STATUS));
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 4)) {
                        log_fusa("BB_ROM_ECC bit irq set.\n");
                        raw_write_op(BB_REG_SYS_ECC_SB_CLR, raw_read_op(BB_REG_SYS_ECC_SB_STATUS));
                        raw_write_op(BB_REG_SYS_ECC_DB_CLR, raw_read_op(BB_REG_SYS_ECC_DB_STATUS));
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 5)) {
                        log_fusa("CPU_ROM_ECC bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 6)) {
                        log_fusa("CPU_SRAM_ECC bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 7)) {
                        log_fusa("OTP_ECC bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 8)) {
                        log_fusa("CAN0_ECC bit irq set.\n");
                        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_ISR_OFFSET,0x00030000);
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 9)) {
                        log_fusa("CAN1_ECC bit irq set.\n");
                        raw_write_op(REL_REGBASE_CAN1 + REG_CAN_ISR_OFFSET,0x00030000);
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 10)) {
                        log_fusa("CPU_WDT bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 11)) {
                        log_fusa("CAN0_ERR bit irq set.\n");
                        //clear CAN0.IR register,write 1 clear
                        value_clear = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_ISR_OFFSET);
                        log_fusa("IR = 0X%x\n",value_clear);
                        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_ISR_OFFSET, value_clear | BIT_INTERRUPT_BEC | BIT_INTERRUPT_BEU);
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 16)) {
                        log_fusa("CAN0_LP bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 13)) {
                        log_fusa("I2C_ACK bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 14)) {
                        log_fusa("CHIRP_LS bit irq set.\n");
                        value_clear = fmcw_radio_switch_bank(NULL,10);
                        fmcw_radio_reg_write(NULL,0x70,0x0d);//clear irq
                        fmcw_radio_reg_write(NULL,0x70,0x0c);
                        fmcw_radio_switch_bank(NULL,value_clear);
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 15)) {
                        log_fusa("XIP_ECC bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 18)) {
                        log_fusa("SPI_LP bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 19)) {
                        log_fusa("SPI_CRC bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else if (value & (1 << 20)) {
                        log_fusa("FLASH_CRC bit irq set.\n");
                        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
                        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
                } else {
                        log_fusa("Unused bit irq set.\n");
                }
        }
        //enable digital error irq
        int_enable(INT_DG_ERROR_IRQ);
}


/****************************************************
 * NAME         : emu_digital_error_ss1_cpu_22_isr
 * DESCRIPTIONS : emu digital error ss1 cpu 22 isr handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void emu_digital_error_ss1_cpu_22_isr(void *params) {
        uint32_t value;
        uint32_t value_clear;
        //uint32_t compare;

        //disable first then enable
        int_disable(INT_DG_ERROR_SS1);

        portENTER_CRITICAL();
        log_fusa("emu digital error ss1 cpu 22 isr.\n");
        // check EMU.DIG_ERR_SS1_STA.FLASH_CRC bit
        value = raw_read_op(REG_EMU_DIG_ERR_SS1_STA);
        log_fusa("check EMU.DIG_ERR_SS1_STA_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_SS1_STA, value);
        //clear the interrupt flag that lead to 22 isr
        value_clear = raw_read_op(REG_EMU_DIG_ERR_CLR);
        //log_fusa("check EMU.REG_EMU_DIG_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_CLR, value_clear);
        raw_write_op(REG_EMU_DIG_ERR_CLR, value | value_clear);
        //chip_hw_mdelay(2000);
        portEXIT_CRITICAL();
        if (value) {
                if (value & (1 << 0)) {
                        //BB_LBIST bit
                        log_fusa(" BB_LBIST bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 3)) {
                        //BB_SRAM_ECC bit
                        log_fusa(" BB_SRAM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 4)) {
                        //BB_ROM_ECC bit
                        log_fusa(" BB_ROM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 5)) {
                        //CPU_ROM_ECC bit
                        log_fusa(" CPU_ROM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 6)) {
                        //CPU_SRAM_ECC bit
                        log_fusa(" CPU_SRAM_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 7)) {
                        //OTP_ECC bit
                        log_fusa(" OTP_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 8)) {
                        //CAN0_ECC bit
                        log_fusa(" CAN0_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 10)) {
                        //CPU_WDT bit
                        log_fusa(" CPU WDT bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 11)) {
                        //CAN0_ERR bit
                        log_fusa(" CAN0_ERR bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 16)) {
                        //CAN0_LP bit
                        log_fusa(" CAN0_LP bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 13)) {
                        //I2C_ACK bit
                        log_fusa(" I2C_ACK bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 14)) {
                        //CHIRP_LS bit
                        log_fusa(" CHIRP_LS bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 15)) {
                        //XIP_ECC bit
                        log_fusa(" XIP_ECC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 18)) {
                        //SPI_LP bit
                        log_fusa(" SPI_LP bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 19)) {
                        //SPI_CRC bit
                        log_fusa(" SPI_CRC bit set EMU.SS1_CTRL to 0x1.\n");
                        raw_write_op(REG_EMU_SS1_CTRL, 1);
                } else if (value & (1 << 20)) {
                        //FLASH_CRC bit
                        //gpio_write(LED_D1_NO, LED_D1_OFF);//end test
                        //set EMU.SS1_CTRL to 0x1
                        log_fusa(" FLASH_CRC bit set EMU.SS1_CTRL to 0x1.\n");
                        //chip_hw_mdelay(2000);
                        value = 0x1;
                        raw_write_op(REG_EMU_SS1_CTRL, value);
                        //value = raw_read_op(REG_EMU_SS1_CTRL);
                        //log_fusa("set EMU.SS1_CTRL to 0x1_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_SS1_CTRL, value);
                } else {
                        //enable digital error irq
                        int_enable(INT_DG_ERROR_SS1);
                }
        }else {
                //enable digital error irq
                int_enable(INT_DG_ERROR_SS1);
        }
}

/****************************************************
 * NAME         : func_safety_set_dmu_irq_0_32_ena
 * DESCRIPTIONS : set dmu irq 0_32 enable
 * INPUT        : bit_shift:bit 0_32 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_dmu_irq_0_32_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set DMU.IRQ_ENA_0_32.bit_shift bit to 0x1
        value = raw_read_op(REL_REGBASE_DMU + REG_DMU_IRQ_ENA0_31_OFFSET);
        raw_write_op(REL_REGBASE_DMU + REG_DMU_IRQ_ENA0_31_OFFSET, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_irq_mask
 * DESCRIPTIONS : set reg emu rf err irq mask enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_irq_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_IRQ_MARK.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_RF_ERR_IRQ_MASK);
        raw_write_op(REG_EMU_RF_ERR_IRQ_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_irq_ena
 * DESCRIPTIONS : set reg emu rf err irq enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_irq_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_IRQ_ENA.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_RF_ERR_IRQ_ENA);
        raw_write_op(REG_EMU_RF_ERR_IRQ_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_irq_mask
 * DESCRIPTIONS : clear reg emu rf err irq mask
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_irq_mask(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_IRQ_MARK.bit_shift bit
        value = raw_read_op(REG_EMU_RF_ERR_IRQ_MASK);
        //log_fusa("irq mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_write_op(REG_EMU_RF_ERR_IRQ_MASK, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_irq_ena
 * DESCRIPTIONS : clear reg emu rf err irq
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_irq_ena(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_IRQ_ENA.bit_shift bit
        value = raw_read_op(REG_EMU_RF_ERR_IRQ_ENA);
        //log_fusa("irq ena value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_write_op(REG_EMU_RF_ERR_IRQ_ENA, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss1_mask
 * DESCRIPTIONS : set reg emu rf err ss1 mask enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss1_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS1_MARK.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_RF_ERR_SS1_MASK);
        raw_write_op(REG_EMU_RF_ERR_SS1_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss1_ena
 * DESCRIPTIONS : set reg emu rf err ss1 enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss1_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS1_ENA.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_RF_ERR_SS1_ENA);
        raw_write_op(REG_EMU_RF_ERR_SS1_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_ss1_mask
 * DESCRIPTIONS : clear reg emu rf err ss1 mask
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_ss1_mask(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS1_MARK.bit_shift bit
        value = raw_read_op(REG_EMU_RF_ERR_SS1_MASK);
        //log_fusa("ss1 mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_write_op(REG_EMU_RF_ERR_SS1_MASK, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_ss1_ena
 * DESCRIPTIONS : clear reg emu rf err ss1
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_ss1_ena(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS1_ENA.bit_shift bit
        value = raw_read_op(REG_EMU_RF_ERR_SS1_ENA);
        //log_fusa("ss1 ena value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_write_op(REG_EMU_RF_ERR_SS1_ENA, value & (~(1 << bit_shift)));
}


/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_irq_mask
 * DESCRIPTIONS : set reg emu dig err irq mask enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_irq_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_IRQ_MARK.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_DIG_ERR_IRQ_MASK);
        raw_write_op(REG_EMU_DIG_ERR_IRQ_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_irq_ena
 * DESCRIPTIONS : set reg emu dig err irq enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_irq_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_IRQ_ENA.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_DIG_ERR_IRQ_ENA);
        raw_write_op(REG_EMU_DIG_ERR_IRQ_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_dig_err_irq_ena
 * DESCRIPTIONS : clear reg emu dig err irq enable
 * INPUT        : bit_shift:bit 0~20 to disable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_dig_err_irq_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_IRQ_ENA.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_DIG_ERR_IRQ_ENA);
        raw_write_op(REG_EMU_DIG_ERR_IRQ_ENA, value & (~(1 << bit_shift)));
}
/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss1_mask
 * DESCRIPTIONS : set reg emu dig err ss1 mask enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss1_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS1_MARK.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_DIG_ERR_SS1_MASK);
        raw_write_op(REG_EMU_DIG_ERR_SS1_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss1_ena
 * DESCRIPTIONS : set reg emu dig err ss1 enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss1_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS1_ENA.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_DIG_ERR_SS1_ENA);
        raw_write_op(REG_EMU_DIG_ERR_SS1_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss2_mask
 * DESCRIPTIONS : set reg emu rf err ss2 mask enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss2_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS2_MARK.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_RF_ERR_SS2_MASK);
        raw_write_op(REG_EMU_RF_ERR_SS2_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_rf_err_ss2_ena
 * DESCRIPTIONS : set reg emu rf err ss2 enable
 * INPUT        : bit_shift:bit 0~12 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_rf_err_ss2_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.RF_ERR_SS2_ENA.bit_shift to 0x1
        value = raw_read_op(REG_EMU_RF_ERR_SS2_ENA);
        raw_write_op(REG_EMU_RF_ERR_SS2_ENA, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_ss2_mask
 * DESCRIPTIONS : clear reg emu rf err ss2 mask
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_ss2_mask(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS2_MARK.bit_shift bit
        value = raw_read_op(REG_EMU_RF_ERR_SS2_MASK);
        //log_fusa("ss2 mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_write_op(REG_EMU_RF_ERR_SS2_MASK, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_clear_reg_emu_rf_err_ss2_ena
 * DESCRIPTIONS : clear reg emu rf err ss2
 * INPUT        : bit_shift:bit 0~12 to clear
 * OUTPUT       : None
 *****************************************************/
static void func_safety_clear_reg_emu_rf_err_ss2_ena(uint8_t bit_shift)
{
        uint32_t value;

        //clear EMU.RF_ERR_SS2_ENA.bit_shift
        value = raw_read_op(REG_EMU_RF_ERR_SS2_ENA);
        //log_fusa("ss2 mask value: 0x%x  valueand: 0x%x  \n", value, value & (~(1 << bit_shift)));
        raw_write_op(REG_EMU_RF_ERR_SS2_ENA, value & (~(1 << bit_shift)));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss2_mask
 * DESCRIPTIONS : set reg emu dig err ss2 mask enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss2_mask(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS2_MARK.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_DIG_ERR_SS2_MASK);
        raw_write_op(REG_EMU_DIG_ERR_SS2_MASK, value | (1 << bit_shift));
}

/****************************************************
 * NAME         : func_safety_set_reg_emu_dig_err_ss2_ena
 * DESCRIPTIONS : set reg emu dig err ss2 enable
 * INPUT        : bit_shift:bit 0~20 to enable
 * OUTPUT       : None
 *****************************************************/
static void func_safety_set_reg_emu_dig_err_ss2_ena(uint8_t bit_shift)
{
        uint32_t value;

        //set EMU.DIG_ERR_SS2_ENA.bit_shift bit to 0x1
        value = raw_read_op(REG_EMU_DIG_ERR_SS2_ENA);
        raw_write_op(REG_EMU_DIG_ERR_SS2_ENA, value | (1 << bit_shift));
}

//spi loopback test parameter initial
static spi_xfer_desc_t fusa_spi_xfer_desc =
{
        .clock_mode = SPI_CLK_MODE_0,
        .dfs = 32,
        .cfs = 0,
        .spi_frf = SPI_FRF_STANDARD,
        .rx_thres = 0,
        .tx_thres = 0,
};

/****************************************************
 * NAME         : func_safety_sm_spi_loopback
 * DESCRIPTIONS : SM126 spi loop init and test
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void func_safety_sm_spi_loopback(uint8_t func_safety_error_type) {
        int32_t result = E_OK;
        uint32_t m0_send[1] = { 0x11223344 };
        uint32_t spi_loop_expect_rx[4] = { 0x11, 0x22, 0x33, 0x44 };
        uint32_t qspi_loop_expect_rx[8] = { 0x12345678, 0x1234567, 0x123456,
                        0x12345, 0x1234, 0x123, 0x12, 0x1 };
        uint32_t slave_read[4] = { 0 };
        uint32_t qspi_slave_read[8] = { 0 };
        uint32_t spi_slave_SCTRLR0_recover_value = 0;
        uint32_t recover_dmu_mux_spi_m1;
        uint32_t recover_dmu_mux_spi_s;
        uint8_t i;
        bool flag = false;

//SPI LOOPBACK TEST
        //set DMU_SYS_SPI_LOOP to 0x1
        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, 1);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET);
        //log_fusa("REG_DMU_SYS_SPI_LOOP_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, value);
        //spi master0 init
        io_mux_spi_m0_func_sel(0);
        if (spi_open(0, 25000000) != E_OK){
                log_fusa("spi_open DW_SPI_0_ID fail.\n");
        }
        if (spi_transfer_config(0, &fusa_spi_xfer_desc) != E_OK){
                log_fusa("transfer_config DW_SPI_0_ID fail.\n");
        }

        //spi slave init
        result = spi_open(2, 25000000);
        if (result != E_OK){
                log_fusa("spi_open DW_SPI_2_ID fail.\n");
        }

        //master0 wtite data
        spi_write(0, m0_send, 1);
        //slave read data
        spi_read(2, slave_read, 4);

        //print data
        //log_fusa("m0_send: 0x%x.\n", m0_send[0]);
        //log_fusa("slave_read: 0x%x  0x%x  0x%x  0x%x.\n", slave_read[0], slave_read[1], slave_read[2], slave_read[3]);
        io_mux_spi_m0_func_sel(4);
        //log_fusa("SPI Loop Test Done!\n", m0_send[0]);
        spi_slave_SCTRLR0_recover_value = raw_read_op(SPI_S_ADDR + CTRLR0);
        //log_fusa("SPI_S_ADDR+CTRLR0_ADDR_READ: 0x%x, value: 0x%x.\n", SPI_S_ADDR+CTRLR0, spi_slave_SCTRLR0_recover_value);
        for (i = 0; i < 4; i++) {
                if (spi_loop_expect_rx[i] != slave_read[i]) {
                        flag = true;
                        break;
                }
        }

//QSPI LOOPBACK TEST
        //PLL enable
        //value = raw_read_op(CLKGEN_ADDR + 0x110);
        //log_fusa("CLKGEN_DIV_APB_REF_ADDR_READ: 0x%x, value: 0x%x.\n", CLKGEN_ADDR + 0x110, value);

        //set DMU_SYS_SPI_LOOP to 0x1
        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, 1);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET);
        //log_fusa("REG_DMU_SYS_SPI_LOOP_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, value);
        //set DMU_MUX_SPI_S to 0x2
        recover_dmu_mux_spi_s = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, 2);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        //log_fusa("REG_DMU_MUX_SPI_S_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, value);
        //set DMU_MUX_SPI_S1_MOSI to 0x2
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, 2);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET);
        //log_fusa("REG_DMU_MUX_SPI_S1_MOSI_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, value);
        //set DMU_MUX_SPI_S1_MISO to 0x2
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, 2);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET);
        //log_fusa("REG_DMU_MUX_SPI_S1_MISO_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, value);

        // config qspi
        raw_write_op(REG_CLKGEN_ENA_QSPI, 1);        //enable QSPI clk
        raw_write_op(REG_CLKGEN_RSTN_QSPI, 0);        //CLKGEN_RSTN_QSPI
        raw_write_op(REG_CLKGEN_RSTN_QSPI, 1);        //CLKGEN_RSTN_QSPI
        raw_write_op(REG_CLKGEN_RSTN_SPI_S, 0);        //CLKGEN_RSTN_SPI_S
        raw_write_op(REG_CLKGEN_RSTN_SPI_S, 1);        //CLKGEN_RSTN_SPI_S

        int SCKDV = 400 / 25;

        recover_dmu_mux_spi_m1 = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET);
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET, 2); //set DMU_MUX_SPI_M1 to QSPI_M1
        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_SSIENR_OFFSET, 0); // disables the QSPI
        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_BAUDR_OFFSET, SCKDV); //Baud Rate divider
        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_CTRLR0_OFFSET,
                        0x005f0000 | ((1 & 0x3) << 8));        //quad spi 32bit transmit
        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_CTRLR1_OFFSET, 0x7ff); //number of data frames
        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_SPI_CTRLR0_OFFSET, 0x00004202); // 8-bit instruction
        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_SSIENR_OFFSET, 1); // enables the QSPI

        // config slave
        raw_write_op(REG_CLKGEN_ENA_SPI_S, 1);        //enable slave clk
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0000); // disables the SPI slave
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_CTRLR0_OFFSET,
                        0x005f0000 | ((2 & 0x3) << 8)); //32 bit, receive , (SCPOL,SCPH)=(0,0)
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_TXFTLR_OFFSET, 0x0001); //tx_num <=1 -> irq on
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_RXFTLR_OFFSET, 0x0001); //rx_num > 0 -> irq on
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_IMR_OFFSET, 0x0000); //all mask  receive fifo full and transmit fifo empty
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0001); // enables the SPI slave

        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_DR_OFFSET(0), 0x80); // write 8-bit instruction
        for (i = 0; i < 8; i++) {
                raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_DR_OFFSET(0),
                                (TEST_QSPI_DAT >> (i * 4)) & 0xffffffff);        //write data
        }

        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_SER_OFFSET, (1 << 0)); //Slave is selected
        //wait QSPI transmit finish
        for (i = 0; i < 2; i++) {
                while ((raw_read_op(REL_REGBASE_QSPI + REG_DW_SSI_SR_OFFSET) & 0x4) != 0x4) { //transmit fifo empty
                        ;
                }
        }

        //wait slave receive data finish
        while (raw_read_op(REL_REGBASE_SPI2 + REG_DW_SSI_RXFLR_OFFSET) != 0x08) {
                ;
        }
        //chip_hw_mdelay(10);
        raw_write_op(REL_REGBASE_QSPI + REG_DW_SSI_SER_OFFSET, 0); //No slave selected
        //read the data
        for (i = 0; i < 8; i++) {
                qspi_slave_read[i] = raw_read_op(
                REL_REGBASE_SPI2 + REG_DW_SSI_DR_OFFSET(0));
        }

        for (i = 0; i < 8; i++) {
                if (qspi_loop_expect_rx[i] != qspi_slave_read[i]) {
                        flag = true;
                        break;
                }
        }

        //recover register
        //set DMU_SYS_SPI_LOOP to 0x0
        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, 0);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET);
        //log_fusa("REG_DMU_SYS_SPI_LOOP_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_SYS_SPI_LOOP_OFFSET, value);
        //set DMU_MUX_SPI_S to 0x0
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, recover_dmu_mux_spi_s);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        //log_fusa("REG_DMU_MUX_SPI_S_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET, value);
        //set DMU_MUX_SPI_S1_MOSI to 0x0
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, 0);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET);
        //log_fusa("REG_DMU_MUX_SPI_S1_MOSI_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET, value);
        //set DMU_MUX_SPI_S1_MISO to 0x0
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, 0);
        //value = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET);
        //log_fusa("REG_DMU_MUX_SPI_S1_MISO_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET, value);
        //recover REG_DMU_MUX_SPI_M1
        raw_write_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET, recover_dmu_mux_spi_m1);

        //recover spi_slave_SCTRLR0 parameter.
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0000); // disables the SPI slave
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_CTRLR0_OFFSET,
                        spi_slave_SCTRLR0_recover_value); // 8 bit, tx&rx , (SCPOL,SCPH)=(0,0)
        raw_write_op(REL_REGBASE_SPI2 + REG_DW_SSI_SSIENR_OFFSET, 0x0001); // enables the SPI slave
        //value = raw_read_op(SPI_S_ADDR+CTRLR0);
        //log_fusa("SPI_S_ADDR+CTRLR0_ADDR_READ: 0x%x, value: 0x%x.\n", SPI_S_ADDR+CTRLR0, value);

        if (flag == true){
                func_safety_error_handler(fusaConfList[SM_INDEX_27].sm_num, fusaConfList[SM_INDEX_27].error_type);
        }
        else {
                //log_fusa("SPI loop and QSPI loop test ok!\n");
        }
}

/****************************************************
 * NAME         : func_safety_sm_can_loopback_init
 * DESCRIPTIONS : can loopback initial
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_can_loopback_init(void) {
        uint32_t value;
        ;
        uint32_t can_send_buf[2] = { 0x04030201, 0x08070605 };

        //read MCR
        value = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //log_fusa("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //set MCR.CFG = 1
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_CFG);
        value = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //log_fusa("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //set MCR.LBACK = 1
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_LBACK);
        value = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //log_fusa("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //set MCR.CFG = 0
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value & (~BIT_MODE_CTRL_CFG));
        value = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //log_fusa("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);

        //send data
        //log_fusa("func_safety_can_send_data: 0x%x  0x%x.\n", can_send_buf[0], can_send_buf[1]);
        //func_safety_can_send_data(FUNC_SAFETY_FRAME_ID, can_send_buf, eDATA_LEN_8);
        can_send_data(CAN_0_ID, FUNC_SAFETY_FRAME_ID, can_send_buf, eDATA_LEN_8);
}

/****************************************************
 * NAME         : func_safety_sm_can_loopback_rx_handler
 * DESCRIPTIONS : can loopback rx handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_can_loopback_rx_handler(uint8_t *data, uint32_t len) {
        uint8_t func_safety_can_expect_rx_data[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
        bool flag = false;
        //uint32_t value;

        //recover can to normal mode
        //config mode
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, BIT_MODE_CTRL_CFG);
        //value = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //log_fusa("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);
        //normal mode
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, 0);
        //value = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);
        //log_fusa("REG_CAN_MODE_CTRL_OFFSET_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value);

        /*log_fusa("can_read: ");
        for (uint8_t i = 0; i < 8; i++) {
                log_fusa("  0x%x", data[i]);
        }
        log_fusa("\r\n");*/

        for (uint8_t i = 0; i < 8; i++) {
                if (data[i] != func_safety_can_expect_rx_data[i]) {
                        flag = true;
                        break;
                }
        }

        if (flag == true) {
                func_safety_error_handler(fusaConfList[SM_INDEX_21].sm_num, fusaConfList[SM_INDEX_21].error_type);
        } else {
                //log_fusa("CAN loopback test ok!\n");
        }
}


/****************************************************
 * NAME         : func_safety_sm8_enable
 * DESCRIPTIONS : enable/disable sm8
 * INPUT        : func_safety_error_type: irq or ss1 or ss2
                  enable_flag: true/false, true: enable, false: disable
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm8_enable(uint8_t func_safety_error_type, bool enable_flag)
{
        uint32_t value;

        switch (func_safety_error_type) {
                case SAFE_STATE_IRQ:
                        if (enable_flag == false) {
                                value = raw_read_op(REG_EMU_RF_ERR_STA);
                                //log_fusa("RF_ERR_STA front: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                                func_safety_clear_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_clear_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        } else {
                                value = raw_read_op(REG_EMU_RF_ERR_CLR);
                                //log_fusa("after read RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value);
                                raw_write_op(REG_EMU_RF_ERR_CLR, value | (1 << FUNC_SAFETY_BIT_7_SHIFT));

                                value = raw_read_op(REG_EMU_RF_ERR_STA);
                                //log_fusa("RF_ERR_STA last: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);

                                func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        }
                        break;
                case SAFE_STATE_SS1:
                        if (enable_flag == false) {
                                value = raw_read_op(REG_EMU_RF_ERR_STA);
                                //log_fusa("RF_ERR_STA front: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                                func_safety_clear_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_clear_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        } else {
                                value = raw_read_op(REG_EMU_RF_ERR_CLR);
                                //log_fusa("after read RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value);
                                raw_write_op(REG_EMU_RF_ERR_CLR, value | (1 << FUNC_SAFETY_BIT_7_SHIFT));

                                value = raw_read_op(REG_EMU_RF_ERR_STA);
                                //log_fusa("RF_ERR_STA last: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);

                                func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        }
                        break;
                case SAFE_STATE_SS2:
                        if (enable_flag == false) {
                                value = raw_read_op(REG_EMU_RF_ERR_STA);
                                //log_fusa("RF_ERR_STA front: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                                func_safety_clear_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_clear_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        } else {
                                value = raw_read_op(REG_EMU_RF_ERR_CLR);
                                //log_fusa("after read RF_ERR_CLR_ADDR: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_CLR, value);
                                raw_write_op(REG_EMU_RF_ERR_CLR, value | (1 << FUNC_SAFETY_BIT_7_SHIFT));

                                value = raw_read_op(REG_EMU_RF_ERR_STA);
                                //log_fusa("RF_ERR_STA last: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);

                                func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                                //func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        }
                        break;
                default:
                        log_fusa("\n");
                        break;
        }
}

/****************************************************
 * NAME         : func_safety_sm_init
 * DESCRIPTIONS : func safety sm init
 * INPUT        : func_safety_sm_num: sm number
 *                func_safety_error_type: ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_init(uint16_t func_safety_sm_num, uint8_t func_safety_error_type)
{
        uint32_t value = 0;
        uint8_t old_bank = 0;
        baseband_t *bb = baseband_get_cur_bb();
#if (SAFETY_FEATURE_SM905 == FEATURE_OFF) && (SAFETY_FEATURE_SM6 == FEATURE_ON)
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(&bb->radio, baseband_t, radio)->cfg;
#endif
        if (func_safety_error_type == SAFE_STATE_IRQ) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM1:
                        //configure radio
                        // IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio);
                        //enable cpu interrupt 19, disable first then enable
#if (SAFETY_FEATURE_SM901 == FEATURE_OFF)
                        fmcw_radio_sm_ldo_monitor_threshold(&bb->radio);
                        old_bank = fmcw_radio_switch_bank(&bb->radio, 10);
                        /* set threshold of LDO Monitor timing */
                        RADIO_WRITE_BANK_REG(10, LDO_MON_TIME_CNT_1, 0x3F);
                        RADIO_WRITE_BANK_REG(10, LDO_MON_CTU_SIZE, 0x18);
                        fmcw_radio_switch_bank(&bb->radio, old_bank);
#endif
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_xxx_MARK.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_ERR_xxx_ENA.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_TEST_ENA.LDO_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 0));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM2:
#if (SAFETY_FEATURE_SM902 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, false);
#endif
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_xxx_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_ERR_xxx_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_TEST_ENA.AVDD33_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 1));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM3:
#if (SAFETY_FEATURE_SM902 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, false);
#endif
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_xxx_MARK.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_ERR_xxx_ENA.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_TEST_ENA.DVDD11_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 2));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM4:
#if (SAFETY_FEATURE_SM904 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, false);
#endif
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_xxx_MARK.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_ERR_xxx_ENA.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 3));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM5:
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_xxx_MARK.PLL_LOCK_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.RF_ERR_xxx_ENA.PLL_LOCK_DT_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //configure radio
                        fmcw_radio_reg_write(NULL, 0, 0);
                        fmcw_radio_reg_write(NULL, 0x7d, 0x15);
                        fmcw_radio_reg_write(NULL, 0, 0xa);
                        fmcw_radio_reg_write(NULL, 0x6e, 0x0);
                        fmcw_radio_reg_write(NULL, 0x6e, 0x2);
                        //set EMU.RF_TEST_ENA.PLL_LOCK_DT bit to 0x1
                        //value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        //raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 4));
                        //wait error status
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM6:
#if (SAFETY_FEATURE_SM905 == FEATURE_OFF) && (SAFETY_FEATURE_SM6 == FEATURE_ON)
                        fmcw_radio_sm_rfpower_detector_threshold(&bb->radio, cfg->fmcw_startfreq, 5, fmcw_radio_check_txon_channel(&bb->radio));
#endif
                        // //configure radio
                        // IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_TEST_ENA.RF_POWER_DT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 5));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                // case FUNC_SAFETY_ITEM_SM7:
                //         //enable cpu interrupt 20, disable first then enable
                //         int_disable(INT_RF_ERROR_SS1);
                //         if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                //                 log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                //         //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                //         func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                //         //set EMU.RF_ERR_SS1_MARK.TX_BALL_DT bit to 0x1
                //         func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_6_SHIFT);
                //         //set EMU.RF_ERR_SS1_ENA.TX_BALL_DT bit to 0x1
                //         func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_6_SHIFT);
                //         //configure radio

                //         //set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1
                //         log_fusa("set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1.\n");
                //         value = raw_read_op(REG_EMU_RF_TEST_ENA);
                //         raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 6));
                //         int_enable(INT_RF_ERROR_SS1);
                //         break;
                case FUNC_SAFETY_ITEM_SM8:
                case FUNC_SAFETY_ITEM_SM9:
                case FUNC_SAFETY_ITEM_SM10:
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        raw_write_op(REG_EMU_RF_ERR_CLR,raw_read_op(REG_EMU_RF_ERR_STA));
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RX_SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RX_SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //clear err_sta


                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM11:
                        //configure radio & bb
                        // IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        //configure radio & bb
                        // IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM13:
                        //configure radio
                        // IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_TEST_ENA.CHIRP_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 8));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM14:
                        // //configure radio
                        // IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, false);
                        // log_fusa("aa1\n");
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        // log_fusa("aa2\n");
                        //set EMU.RF_ERR_SS1_MARK.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_TEST_ENA.TEMP_OVER bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 9));
                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        //enable cpu interrupt 19, disable first then enable
                        int_disable(INT_RF_ERROR_IRQ);
                        if (int_handler_install(INT_RF_ERROR_IRQ, emu_rf_error_irq_cpu_19_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_irq_init_error: %d.\n", INT_RF_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.19 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_mask(FUNC_SAFETY_BIT_12_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_12_SHIFT);
                        //configure radio & bb

                        //enable test and check results

                        int_enable(INT_RF_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM101:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //configure radio

                        /*//set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set EMU.BB_LBIST_ENA to 0x1
                        raw_write_op(REG_EMU_BB_LBIST_ENA, 0x1);*/
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM102:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        core_reg_write(REG_CPU_ERP_CTRL, 0xf);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM103:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        core_reg_write(REG_CPU_ERP_CTRL, 0xf);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM104:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_write_op(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM105:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_write_op(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM106:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set DMU.MEMINI_ENA to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM107:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //set DMU.MEMRUN_ENA to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM108:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //set DMU.OTP_ECC_EN to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM109:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
#if SAFETY_FEATURE_CAN0 ==  FEATURE_ON
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        func_safety_enable_can_ecc(0);
#endif
#if SAFETY_FEATURE_CAN1 ==  FEATURE_ON
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can1_enable(1);
                        func_safety_enable_can_ecc(1);
#endif
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_16_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_16_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //configure can

                        //enable test and check results
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        // initialize can, done in main()

                        // set CAN_0.IE.PEDE to 0x1
                        value = raw_read_op(0xBB0000 + 0x0038);
                        raw_write_op(0xBB0000 + 0x0038, value | (1 << 23));
                        //set CAN_0.ILS0R.PEDELS to 0x2
                        value = raw_read_op(0xBB0000 + 0x003C);
                        raw_write_op(0xBB0000 + 0x003C, value | (1 << 17));
                        //set CAN_0.ILE.EINT2 to 0x1
                        value = raw_read_op(0xBB0000 + 0x0044);
                        raw_write_op(0xBB0000 + 0x0044, value | (1 << 2));
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_18_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_18_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_14_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_14_SHIFT);
                        // configure radio
                        old_bank = fmcw_radio_switch_bank(NULL,10);
                        fmcw_radio_reg_write(NULL,0x70,0x08);//enable lockstep
                        fmcw_radio_reg_write(NULL,0x70,0x0c);//release IRQ reset
                        fmcw_radio_reg_write(NULL,0x70,0x0e);//start lockstep
                        fmcw_radio_reg_write(NULL,0x70,0x0c);//start lockstep
                        fmcw_radio_reg_write(NULL,0x70,0x0d);//clear irq
                        fmcw_radio_reg_write(NULL,0x70,0x0c);
                        fmcw_radio_switch_bank(NULL,old_bank);
                        // // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        // value = raw_read_op(REG_EMU_DIG_TEST_ENA);
                        // raw_write_op(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_15_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_15_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                case FUNC_SAFETY_ITEM_SM206:
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //enable cpu interrupt 21, disable first then enable
                        int_disable(INT_DG_ERROR_IRQ);
                        if (int_handler_install(INT_DG_ERROR_IRQ, emu_digital_error_irq_cpu_21_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_IRQ_init_error: %d.\n", INT_DG_ERROR_IRQ);
                        }
                        //set DMU.IRQ_ENA_0_32.21 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_21_SHIFT);
                        //set EMU.DIG_ERR_IRQ_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_IRQ_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_IRQ);
                        break;
                default:
                        log_fusa("\n");
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS1) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM1:
                        //configure radio
                        // IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio);
                        //enable cpu interrupt 20, disable first then enable
#if (SAFETY_FEATURE_SM901 == FEATURE_OFF)
                        fmcw_radio_sm_ldo_monitor_threshold(&bb->radio);
                        uint8_t old_bank = fmcw_radio_switch_bank(&bb->radio, 10);
                        /* set threshold of LDO Monitor timing */
                        RADIO_WRITE_BANK_REG(10, LDO_MON_TIME_CNT_1, 0x3F);
                        RADIO_WRITE_BANK_REG(10, LDO_MON_CTU_SIZE, 0x18);
                        fmcw_radio_switch_bank(&bb->radio, old_bank);
#endif
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1,
                                        emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n",
                                                INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_TEST_ENA.LDO_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 0));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM2:
#if (SAFETY_FEATURE_SM902 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, false);
#endif
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_TEST_ENA.AVDD33_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 1));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM3:
#if (SAFETY_FEATURE_SM902 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, false);
#endif
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_xxx_MARK.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_ERR_xxx_ENA.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_TEST_ENA.DVDD11_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 2));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM4:
#if (SAFETY_FEATURE_SM904 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, false);
#endif
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_xxx_MARK.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_ERR_xxx_ENA.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_TEST_ENA.VBG_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 3));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM5:
                        //set EMU.RF_ERR_SS1_MARK.PLL_LOCK_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.PLL_LOCK_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //configure radio
                        fmcw_radio_reg_write(NULL, 0, 0);
                        fmcw_radio_reg_write(NULL, 0x7d, 0x15);
                        fmcw_radio_reg_write(NULL, 0, 0xa);
                        fmcw_radio_reg_write(NULL, 0x6e, 0x0);
                        fmcw_radio_reg_write(NULL, 0x6e, 0x2);
                        //set EMU.RF_TEST_ENA.PLL_LOCK_DT bit to 0x1
                        //value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        //raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 4));
                        //wait error status
                        break;
                case FUNC_SAFETY_ITEM_SM6:
#if (SAFETY_FEATURE_SM905 == FEATURE_OFF) && (SAFETY_FEATURE_SM6 == FEATURE_ON)
                        fmcw_radio_sm_rfpower_detector_threshold(&bb->radio, cfg->fmcw_startfreq, 5, fmcw_radio_check_txon_channel(&bb->radio));
#endif
                        // //configure radio
                        // IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RF_POWER_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RF_POWER_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_TEST_ENA.RF_POWER_DT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 5));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                // case FUNC_SAFETY_ITEM_SM7:
                //         //enable cpu interrupt 20, disable first then enable
                //         int_disable(INT_RF_ERROR_SS1);
                //         if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK)
                //                 log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                //         //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                //         func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                //         //set EMU.RF_ERR_SS1_MARK.TX_BALL_DT bit to 0x1
                //         func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_6_SHIFT);
                //         //set EMU.RF_ERR_SS1_ENA.TX_BALL_DT bit to 0x1
                //         func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_6_SHIFT);
                //         //configure radio

                //         //set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1
                //         log_fusa("set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1.\n");
                //         value = raw_read_op(REG_EMU_RF_TEST_ENA);
                //         raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 6));
                //         int_enable(INT_RF_ERROR_SS1);
                //         break;
                case FUNC_SAFETY_ITEM_SM8:
                case FUNC_SAFETY_ITEM_SM9:
                case FUNC_SAFETY_ITEM_SM10:
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //configure radio & bb

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM11:
                        //configure radio & bb
                        // IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        //configure radio & bb
                        // IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //enable test and check results

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM13:
                        //configure radio
                        // IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_TEST_ENA.CHIRP_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 8));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM14:
                        // //configure radio
                        // IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, false);
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_TEST_ENA.TEMP_OVER bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 9));
                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        //enable cpu interrupt 20, disable first then enable
                        int_disable(INT_RF_ERROR_SS1);
                        if (int_handler_install(INT_RF_ERROR_SS1, emu_rf_error_ss1_cpu_20_isr) != E_OK){
                                log_fusa("INT_RF_ERROR_SS1_init_error: %d.\n", INT_RF_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.20 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.RF_ERR_SS1_MARK.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_mask(FUNC_SAFETY_BIT_12_SHIFT);
                        //set EMU.RF_ERR_SS1_ENA.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss1_ena(FUNC_SAFETY_BIT_12_SHIFT);
                        //configure radio & bb

                        //enable test and check results

                        int_enable(INT_RF_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM101:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //configure radio

                        /*//set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set EMU.BB_LBIST_ENA to 0x1
                        raw_write_op(REG_EMU_BB_LBIST_ENA, 0x1);*/
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM102:
                        int_disable(INT_DG_ERROR_SS1);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        core_reg_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM103:
                        int_disable(INT_DG_ERROR_SS1);
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        core_reg_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM104:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_write_op(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM105:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_write_op(BB_REG_SYS_ECC_ENA, 0x7fff);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM106:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set DMU.MEMINI_ENA to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM107:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //set DMU.MEMRUN_ENA to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM108:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //set DMU.OTP_ECC_EN to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET, 0x01);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM109:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
#if SAFETY_FEATURE_CAN0 ==  FEATURE_ON
                        //set EMU.DIG_ERR_SS1_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        func_safety_enable_can_ecc(0);
#endif
#if SAFETY_FEATURE_CAN1 ==  FEATURE_ON
                        //set EMU.DIG_ERR_SS1_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can1_enable(1);
                        func_safety_enable_can_ecc(1);
#endif
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM113:
                        //set EMU.DIG_ERR_SS1_MARK.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable cpu watchdog timer, period and event_timeou

                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_16_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_16_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //configure can

                        //enable test and check results
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        // initialize can, done in main()

                        // set CAN_0.IE.PEDE to 0x1
                        value = raw_read_op(0xBB0000 + 0x0038);
                        raw_write_op(0xBB0000 + 0x0038, value | (1 << 23));
                        //set CAN_0.ILS0R.PEDELS to 0x2
                        value = raw_read_op(0xBB0000 + 0x003C);
                        raw_write_op(0xBB0000 + 0x003C, value | (1 << 17));
                        //set CAN_0.ILE.EINT2 to 0x1
                        value = raw_read_op(0xBB0000 + 0x0044);
                        raw_write_op(0xBB0000 + 0x0044, value | (1 << 2));
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_18_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_18_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_19_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;

                case FUNC_SAFETY_ITEM_SM129:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        int_enable(INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_14_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_14_SHIFT);
                        // configure radio

                        // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        value = raw_read_op(REG_EMU_DIG_TEST_ENA);
                        raw_write_op(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        int_enable(INT_DG_ERROR_SS1);
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_15_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_15_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                case FUNC_SAFETY_ITEM_SM206:
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //enable cpu interrupt 22, disable first then enable
                        int_disable(INT_DG_ERROR_SS1);
                        if (int_handler_install(INT_DG_ERROR_SS1, emu_digital_error_ss1_cpu_22_isr) != E_OK){
                                log_fusa("INT_DG_ERROR_SS1_init_error: %d.\n", INT_DG_ERROR_SS1);
                        }
                        //set DMU.IRQ_ENA_0_32.22 bit to 0x1
                        func_safety_set_dmu_irq_0_32_ena(FUNC_SAFETY_BIT_22_SHIFT);
                        //set EMU.DIG_ERR_SS1_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS1_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss1_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        int_enable(INT_DG_ERROR_SS1);
                        break;
                default:
                        log_fusa("\n");
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS2) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM1:
                        //configure radio
                        // IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio);
                        // if (IRQ_value) {
                        //         log_fusa("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);
                        // }
#if (SAFETY_FEATURE_SM901 == FEATURE_OFF)
                        fmcw_radio_sm_ldo_monitor_threshold(&bb->radio);
                        uint8_t old_bank = fmcw_radio_switch_bank(&bb->radio, 10);
                        /* set threshold of LDO Monitor timing */
                        RADIO_WRITE_BANK_REG(10, LDO_MON_TIME_CNT_1, 0x3F);
                        RADIO_WRITE_BANK_REG(10, LDO_MON_CTU_SIZE, 0x18);
                        fmcw_radio_switch_bank(&bb->radio, old_bank);
#endif
                        //set EMU.RF_ERR_SS2_MARK.LDO_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.LDO_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //set EMU.RF_TEST_ENA.LDO_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 0));
                        break;
                case FUNC_SAFETY_ITEM_SM2:
                        //configure radio
#if (SAFETY_FEATURE_SM902 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, false);
#endif
                        //set EMU.RF_ERR_SS2_MARK.AVDD33_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.AVDD33_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        //set EMU.RF_TEST_ENA.AVDD33_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 1));
                        break;
                case FUNC_SAFETY_ITEM_SM3:
                        //configure radio
#if (SAFETY_FEATURE_SM902 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, false);
#endif
                        //set EMU.RF_ERR_xxx_MARK.DVDD11_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        // set EMU.RF_ERR_xxx_ENA.DVDD11_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        //set EMU.RF_TEST_ENA.DVDD11_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 2));
                        break;
                case FUNC_SAFETY_ITEM_SM4:
                        //configure radio
#if (SAFETY_FEATURE_SM904 == FEATURE_OFF)
                        //configure radio
                        fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, false);
#endif
                        //set EMU.RF_ERR_xxx_MARK.VBG_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        // set EMU.RF_ERR_xxx_ENA.VBG_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set EMU.RF_TEST_ENA.VBG_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 3));
                        break;
                case FUNC_SAFETY_ITEM_SM5:
                        //set EMU.RF_ERR_SS2_MARK.PLL_LOCK_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.PLL_LOCK_DT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //configure radio
                        fmcw_radio_reg_write(NULL, 0, 0);
                        fmcw_radio_reg_write(NULL, 0x7d, 0x15);
                        fmcw_radio_reg_write(NULL, 0, 0xa);
                        fmcw_radio_reg_write(NULL, 0x6e, 0x0);
                        fmcw_radio_reg_write(NULL, 0x6e, 0x2);
                        //set EMU.RF_TEST_ENA.PLL_LOCK_DT bit to 0x1
                        //value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        //raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 4));
                        break;
                case FUNC_SAFETY_ITEM_SM6:
#if (SAFETY_FEATURE_SM905 == FEATURE_OFF) && (SAFETY_FEATURE_SM6 == FEATURE_ON)
                        fmcw_radio_sm_rfpower_detector_threshold(&bb->radio, cfg->fmcw_startfreq, 5, fmcw_radio_check_txon_channel(&bb->radio));
#endif
                        // //configure radio
                        // IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(
                        //                 &bb->radio);
                        // if (IRQ_value) {
                        //         log_fusa("SM6 RF Power Detector IRQ_Value =%d\n", IRQ_value);
                        // }
                        //set EMU.RF_ERR_SS2_MARK.RF_POWER_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.RF_POWER_DT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set EMU.RF_TEST_ENA.RF_POWER_DT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 5));
                        break;
                // case FUNC_SAFETY_ITEM_SM7:
                //         //set EMU.RF_ERR_SS2_MARK.TX_BALL_DT bit to 0x1
                //         func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_6_SHIFT);
                //         // set EMU.RF_ERR_SS2_ENA.TX_BALL_DT to 0x1
                //         func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_6_SHIFT);
                //         //configure radio

                //         //set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1
                //         log_fusa("set EMU.RF_TEST_ENA.TX_BALL_DT bit to 0x1.\n");
                //         value = raw_read_op(REG_EMU_RF_TEST_ENA);
                //         raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 6));
                //         break;
                case FUNC_SAFETY_ITEM_SM8:
                case FUNC_SAFETY_ITEM_SM9:
                case FUNC_SAFETY_ITEM_SM10:
                        //set EMU.RF_ERR_SS2_MARK.SAT_DT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.SAT_DT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //configure radio & bb

                        break;
                case FUNC_SAFETY_ITEM_SM11:
                        //configure radio & bb
                        // IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, false);
                        // if (IRQ_value)
                        //         log_fusa("SM11 IF Loopback IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.IF_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.IF_LP to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        // IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, false);
                        // if (IRQ_value)
                        //         log_fusa("SM12 RF Loopback IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.RX_LP bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.RX_LP to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM13:
                        //configure radio
                        // IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
                        // if (IRQ_value)
                        //         log_fusa("SM13 Chirp Monitor IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.CHIRP_MT bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.CHIRP_MT to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set EMU.RF_TEST_ENA.CHIRP_MT bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 8));
                        break;
                case FUNC_SAFETY_ITEM_SM14:
                        // //configure radio
                        // IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, false);
                        // if (IRQ_value)
                        //         log_fusa("SM14 Over Temp Detector IRQ = %d\n", IRQ_value);
                        //set EMU.RF_ERR_SS2_MARK.TEMP_OVER bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.TEMP_OVERbit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set EMU.RF_TEST_ENA.TEMP_OVER bit to 0x1
                        value = raw_read_op(REG_EMU_RF_TEST_ENA);
                        raw_write_op(REG_EMU_RF_TEST_ENA, value | (1 << 9));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        //set EMU.RF_ERR_SS2_MARK.GAIN_CK bit to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_mask(FUNC_SAFETY_BIT_12_SHIFT);
                        // set EMU.RF_ERR_SS2_ENA.GAIN_CK to 0x1
                        func_safety_set_reg_emu_rf_err_ss2_ena(FUNC_SAFETY_BIT_12_SHIFT);
                        //configure radio & bb

                        //enable test and check results

                        break;
                case FUNC_SAFETY_ITEM_SM101:
                        //set EMU.DIG_ERR_SS2_MARK.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_0_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.BB_LBIST bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        //configure radio

                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set EMU.BB_LBIST_ENA to 0x1
                        raw_write_op(REG_EMU_BB_LBIST_ENA, 0x1);
                        break;
                case FUNC_SAFETY_ITEM_SM102:
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        core_reg_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS2_MARK.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_1_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_CCM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_1_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM103:
                        //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                        core_reg_write(REG_CPU_ERP_CTRL, 0xf);
                        //set EMU.DIG_ERR_SS2_MARK.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_2_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_CACHE_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_2_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM104:
                        //set EMU.DIG_ERR_SS2_MARK.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_3_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.BB_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_3_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_write_op(BB_REG_SYS_ECC_ENA, 0x7fff);
                        break;
                case FUNC_SAFETY_ITEM_SM105:
                        //set EMU.DIG_ERR_SS2_MARK.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_4_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.BB_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_4_SHIFT);
                        //set CLKGEN.ENA_BB_TOP to 0x1
                        bb_top_enable(1);
                        //set CLKGEN.ENA_BB_CORE to 0x1
                        bb_core_enable(1);
                        //set BB.CFG_SYS_ECC_ENA to 0x7fff
                        raw_write_op(BB_REG_SYS_ECC_ENA, 0x7fff);
                        break;
                case FUNC_SAFETY_ITEM_SM106:
                        //set EMU.DIG_ERR_SS2_MARK.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_5_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_ROM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_5_SHIFT);
                        //set DMU.MEMINI_ENA to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET, 0x01);
                        break;
                case FUNC_SAFETY_ITEM_SM107:
                        //set EMU.DIG_ERR_SS2_MARK.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_6_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_SRAM_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_6_SHIFT);
                        //set DMU.MEMRUN_ENA to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET, 0x01);
                        break;
                case FUNC_SAFETY_ITEM_SM108:
                        //set EMU.DIG_ERR_SS2_MARK.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_7_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.OTP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_7_SHIFT);
                        //set DMU.OTP_ECC_EN to 0x1
                        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET, 0x01);
                        break;
                case FUNC_SAFETY_ITEM_SM109:
#if SAFETY_FEATURE_CAN0 ==  FEATURE_ON
                        //set EMU.DIG_ERR_SS2_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_8_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_8_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        func_safety_enable_can_ecc(0);
#endif
#if SAFETY_FEATURE_CAN1 ==  FEATURE_ON
                        //set EMU.DIG_ERR_SS2_MARK.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_9_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CAN0_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_9_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can1_enable(1);
                        func_safety_enable_can_ecc(1);
#endif
                        break;
                case FUNC_SAFETY_ITEM_SM113:
                        //set EMU.DIG_ERR_SS2_MARK.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_10_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CPU_WDT bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_10_SHIFT);
                        //enable cpu watchdog timer, period and event_timeout

                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        //set EMU.DIG_ERR_SS2_MARK.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_16_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CAN0_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_16_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        //configure can
                        //enable test and check results
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        //set EMU.DIG_ERR_SS2_MARK.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_11_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CAN0_ERR bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_11_SHIFT);
                        //set CLKGEN.ENA_CAN_0 to 0x1
                        can0_enable(1);
                        // initialize can, done in main()
                        // set CAN_0.IE.PEDE to 0x1
                        value = raw_read_op(0xBB0000 + 0x0038);
                        raw_write_op(0xBB0000 + 0x0038, value | (1 << 23));
                        //set CAN_0.ILS0R.PEDELS to 0x2
                        value = raw_read_op(0xBB0000 + 0x003C);
                        raw_write_op(0xBB0000 + 0x003C, value | (1 << 17));
                        //set CAN_0.ILE.EINT2 to 0x1
                        value = raw_read_op(0xBB0000 + 0x0044);
                        raw_write_op(0xBB0000 + 0x0044, value | (1 << 2));
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        //set EMU.DIG_ERR_SS2_MARK.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_18_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.SPI_LP bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_18_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        //set EMU.DIG_ERR_SS2_MARK.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_19_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.SPI_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_19_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        //set EMU.DIG_ERR_SS2_MARK.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_13_SHIFT);
                        //set EMU.DIG_ERR_SS2_ENA.I2C_ACK bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_13_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //set EMU.DIG_ERR_SS2_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS2_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        //set EMU.DIG_ERR_SS2_MARK.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_14_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.CHIRP_LS bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_14_SHIFT);
                        // configure radio

                        // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        value = raw_read_op(REG_EMU_DIG_TEST_ENA);
                        raw_write_op(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //set EMU.DIG_ERR_SS2_MARK.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_15_SHIFT);
                        // set EMU.DIG_ERR_SS2_ENA.XIP_ECC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_15_SHIFT);
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                case FUNC_SAFETY_ITEM_SM206:
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //set EMU.DIG_ERR_SS2_MARK.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_mask(FUNC_SAFETY_BIT_20_SHIFT);
                        //set EMU.DIG_ERR_SS2_ENA.FLASH_CRC bit to 0x1
                        func_safety_set_reg_emu_dig_err_ss2_ena(FUNC_SAFETY_BIT_20_SHIFT);
                        break;
                default:
                        log_fusa("\n");
                        break;
                }
        } else {
                log_fusa("\n");
        }
}

/****************************************************
 * NAME         : func_safety_error_handler
 * DESCRIPTIONS : func safety error handler
 * INPUT        : func_safety_sm_num, test num
 *                func_safety_error_type, ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
void func_safety_error_handler(uint16_t func_safety_sm_num,uint8_t func_safety_error_type) {
        uint32_t value;
        //uint8_t IRQ_value;
        //baseband_t *bb = baseband_get_cur_bb();

        if (func_safety_error_type == SAFE_STATE_IRQ) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM11:
                        // set EMU.RF_ERR_STA.IF_LP bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.IF_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 10));
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        // set EMU.RF_ERR_STA.RX_LP bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.RX_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 11));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        // set EMU.RF_ERR_STA.GAIN_CK bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.GAIN_CK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 12));
                        break;
                case FUNC_SAFETY_ITEM_SM110:
                        //write SM110 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 7));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        // set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 16));
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        // set EMU.DIG_ERR_STA.SPI_LP bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.SPI_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 18));
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        // set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 19));
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        //write SM128 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 9));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //write SM129 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 10));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        // set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 15));
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                        //write SM202 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 1));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM203:
                        //write SM203 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 2));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM204:
                        //write SM204 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 3));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM205:
                        break;
                case FUNC_SAFETY_ITEM_SM206:
                        //write SM206 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 5));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM207:
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //log_fusa("EMU.DIG_ERR_STA_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        //log_fusa("EMU.DIG_ERR_STA_READ: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value);
                        //wait cpu interrupt 22
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //write SM805 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 11));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                default:
                        log_fusa("\n");
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS1) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM11:
                        // set EMU.RF_ERR_STA.IF_LP bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.IF_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 10));
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        // set EMU.RF_ERR_STA.RX_LP bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.RX_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 11));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        // set EMU.RF_ERR_STA.GAIN_CK bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.GAIN_CK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 12));
                        break;
                case FUNC_SAFETY_ITEM_SM110:
                        //write SM110 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 7));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        // set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 16));
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        // set EMU.DIG_ERR_STA.SPI_LP bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.SPI_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 18));
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        // set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 19));
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        //write SM128 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 9));
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //write SM129 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 10));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        // set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 15));
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                        //write SM202 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 1));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM203:
                        //write SM203 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 2));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM204:
                        //write SM204 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 3));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM205:
                        break;
                case FUNC_SAFETY_ITEM_SM206:
                        //write SM206 ss1 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 5));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                case FUNC_SAFETY_ITEM_SM207:
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //log_fusa("EMU.DIG_ERR_STA_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        //log_fusa("EMU.DIG_ERR_STA_READ: 0x%x, value: 0x%x.\n", REG_EMU_DIG_ERR_STA, value);
                        //wait cpu interrupt 22
                        break;
                case FUNC_SAFETY_ITEM_SM805:
                        //write SM805 error code bit
                        value = raw_read_op(REG_EMU_SPARED_0);
                        raw_write_op(REG_EMU_SPARED_0, value | (1 << 11));
                        // set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        break;
                default:
                        log_fusa("\n");
                        break;
                }
        } else if (func_safety_error_type == SAFE_STATE_SS2) {
                switch (func_safety_sm_num) {
                case FUNC_SAFETY_ITEM_SM11:
                        // set EMU.RF_ERR_STA.IF_LP bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.IF_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 10));
                        break;
                case FUNC_SAFETY_ITEM_SM12:
                        // set EMU.RF_ERR_STA.RX_LP bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.RX_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 11));
                        break;
                case FUNC_SAFETY_ITEM_SM201:
                        // set EMU.RF_ERR_STA.GAIN_CK bit to 0x1
                        log_fusa("set EMU.RF_ERR_STA.GAIN_CK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_RF_ERR_STA);
                        raw_write_op(REG_EMU_RF_ERR_STA, value | (1 << 12));
                        break;
                case FUNC_SAFETY_ITEM_SM120:
                        // set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.CAN0_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 16));
                        break;
                case FUNC_SAFETY_ITEM_SM121:
                case FUNC_SAFETY_ITEM_SM122:
                case FUNC_SAFETY_ITEM_SM123:
                case FUNC_SAFETY_ITEM_SM124:
                case FUNC_SAFETY_ITEM_SM125:
                        break;
                case FUNC_SAFETY_ITEM_SM126:
                        // set EMU.DIG_ERR_STA.SPI_LP bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.SPI_LP bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 18));
                        break;
                case FUNC_SAFETY_ITEM_SM127:
                        // set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.SPI_CRC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 19));
                        break;
                case FUNC_SAFETY_ITEM_SM128:
                        // set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.I2C_ACK bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 13));
                        break;
                case FUNC_SAFETY_ITEM_SM129:
                        //set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //wait enter SS2 state
                        break;
                case FUNC_SAFETY_ITEM_SM130:
                        // set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1
                        log_fusa("set EMU.DIG_TEST_ENA.CHIRP_LS bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_TEST_ENA);
                        raw_write_op(REG_EMU_DIG_TEST_ENA, value | (1 << 14));
                        break;
                case FUNC_SAFETY_ITEM_SM133:
                        //set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1
                        log_fusa("set EMU.DIG_ERR_STA.XIP_ECC bit to 0x1.\n");
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 15));
                        //enter SS2 state
                        break;
                case FUNC_SAFETY_ITEM_SM202:
                case FUNC_SAFETY_ITEM_SM203:
                case FUNC_SAFETY_ITEM_SM204:
                case FUNC_SAFETY_ITEM_SM205:
                case FUNC_SAFETY_ITEM_SM206:
                case FUNC_SAFETY_ITEM_SM207:
                case FUNC_SAFETY_ITEM_SM805:
                        //set EMU.DIG_ERR_STA.FLASH_CRC bit to 0x1
                        value = raw_read_op(REG_EMU_DIG_ERR_STA);
                        raw_write_op(REG_EMU_DIG_ERR_STA, value | (1 << 20));
                        //wait enter SS2 state
                        break;
                default:
                        log_fusa("\n");
                        break;
                }
        } else {
                log_fusa("\n");
        }
}


/****************************************************
 * NAME         : func_safety_check_emu_error_code
 * DESCRIPTIONS : check reboot cnt and ss1 error code
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_check_error_code(void)
{
        uint8_t i = 0;
        log_fusa("==============functy safety startup err code check====================\n");
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_REBOOT_CNT);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_REBOOT_LIMIT);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_RF_ERR_SS1_CODE0);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_RF_ERR_SS1_CODE1);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_RF_ERR_SS1_CODE2);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_RF_ERR_SS1_CODE3);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_DIG_ERR_SS1_CODE0);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_DIG_ERR_SS1_CODE1);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_DIG_ERR_SS1_CODE2);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_DIG_ERR_SS1_CODE3);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_RF_ERR_SS2_CODE);
        fusa_power_on_check_status[i++] = raw_read_op(REG_EMU_DIG_ERR_SS2_CODE);

        i = 0;
        log_fusa("REG_EMU_REBOOT_CNT            = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_REBOOT_LIMIT          = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_RF_ERR_SS1_CODE0      = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_RF_ERR_SS1_CODE1      = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_RF_ERR_SS1_CODE2      = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_RF_ERR_SS1_CODE3      = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_DIG_ERR_SS1_CODE0     = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_DIG_ERR_SS1_CODE1     = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_DIG_ERR_SS1_CODE2     = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_DIG_ERR_SS1_CODE3     = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_RF_ERR_SS2_CODE       = 0x%08x\n", fusa_power_on_check_status[i++]);
        log_fusa("REG_EMU_DIG_ERR_SS2_CODE      = 0x%08x\n", fusa_power_on_check_status[i++]);

        log_fusa("===================================================================\n");
}


/****************************************************
 * NAME         : fusa_on_time_error_pin_check
 * DESCRIPTIONS : SM205 On Time Error Pin Check,
 *                write 0 or 1 to EMU_ERROR_OUT and EMU_SAFE_STATE,
 *                then confirm the output level
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
static void fusa_on_time_error_pin_check(void) {
        static uint8_t runcount = 1;

        raw_write_op(REG_EMU_SAFETY_KEY1, 1); //Set the register EMU_SAFETY_KEY1 to a non-zero value.
        raw_write_op(REG_EMU_SAFETY_KEY2, 1); //Set the register EMU_SAFETY_KEY2 to the same value with EMU_SAFETY_KEY1.
        raw_write_op(REG_EMU_SAFETY_PAD_CTRL, 1); //Set the register EMU_SAFETY_PAD_CTRL to 1.

        if (runcount % 2) {
                raw_write_op(REG_EMU_ERROR_OUT, 0); //Change the ERROR pin output value by setting the register EMU_ERROR_OUT_O to 0 or 1.
                raw_write_op(REG_EMU_SAFE_STATE, 0); //Change the SAFE_STATE pin output value by setting the register EMU_SAFE_STATE_O to 0 or 1.
        } else {
                raw_write_op(REG_EMU_ERROR_OUT, 1);
                raw_write_op(REG_EMU_SAFE_STATE, 1);
        }

        runcount++;
}

/****************************************************
 * NAME         : func_safety_init
 * DESCRIPTIONS : fucntional safety test item(sm) initial
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_init(func_safety_t *fusa)
{
        uint8_t i = 0;
        log_fusa("%s enter!\n",__func__);
        //used to detect Pulse Width by toggle gpio led_d1.
        led_d1_init();
        gpio_write(LED_D1_NO, LED_D1_OFF);        //output low level

        //enable reboot limit
        raw_write_op(REG_EMU_REBOOT_LIMIT, 0x3);

#if (INTER_FRAME_POWER_SAVE == 1)
        baseband_t *bb = baseband_get_bb(0);
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        for (i = 0; i < FUSA_CONF_MAX; ++i)
        {
                if (fusaConfList[i].open_flag == true)
                {
                        log_fusa("sm init index =%d ,current time =%f\n",i,fusa_get_current_time_ms());
                        func_safety_sm_init(fusaConfList[i].sm_num, fusaConfList[i].error_type);
                }

        }
#if (FUNC_SAFETY_CLI == FEATURE_ON)
        FreeRTOS_CLIRegisterCommand(&fusa_command);
#endif

#if (INTER_FRAME_POWER_SAVE == 1)
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        log_fusa("%s end!\n",__func__);
}

/****************************************************
 * NAME         : func_safety_test_handler
 * DESCRIPTIONS : used to start one item test manually
 * INPUT        : func_safety_sm_num: sm number, func_safety_error_type: error type, ss1 or ss2.
 * OUTPUT       : None
 *****************************************************/
void func_safety_test_handler(int32_t func_safety_sm_num, uint8_t func_safety_error_type)
{
// #if (FUNC_SAFETY_CLI == 1)
#if 0
        uint32_t value;
        uint32_t IRQ_value;
        bool ret;
        baseband_t *bb = baseband_get_cur_bb();
        sensor_config_t *cfg = sensor_config_get_cur_cfg();
        //static uint8_t cnt = 1;
        //runtime test
        /*float time_calc_t1 = 0.0;
        float time_calc_t2 = 0.0;
        float time_calc_t12 = 0.0;*/

        switch (func_safety_sm_num) {
        case FUNC_SAFETY_ITEM_SM1:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_ldo_monitor_fault_injection_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm1_runtime = %6.4f\n", time_calc_t12);*/

                /*if(cnt%2){
                 IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio);
                 log_fusa("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);
                 }else{
                 IRQ_value = fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio);
                 log_fusa("SM1 LDO Monitor IRQ_Value =%d\n", IRQ_value);
                 }
                 cnt++;*/
                break;
        case FUNC_SAFETY_ITEM_SM2:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM2 AVDD33 Monitor IRQ = %d\n", IRQ_value);
                break;
        case FUNC_SAFETY_ITEM_SM3:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM3 DVDD11 Monitor IRQ = %d\n", IRQ_value);
                break;
        case FUNC_SAFETY_ITEM_SM4:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM4 BG Monitor IRQ = %d\n", IRQ_value);
                break;
        case FUNC_SAFETY_ITEM_SM5:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                fmcw_radio_reg_write(NULL, 0, 0);
                fmcw_radio_reg_write(NULL, 125, 0x14);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                break;
        case FUNC_SAFETY_ITEM_SM6:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rfpower_detector_fault_injection_IRQ(&bb->radio, 5);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM6 RF Power Detector IRQ_Value =%d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm6_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM7:
                {
                        uint32_t ECC_ENABLE = raw_read_op(BB_REG_SYS_ECC_ENA);
                        uint32_t ECC_SB_STAT = raw_read_op(BB_REG_SYS_ECC_SB_STATUS);
                        uint32_t ECC_DB_STAT =  raw_read_op(BB_REG_SYS_ECC_DB_STATUS);
                        log_fusa("ECC_ENABLE_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_ENA, ECC_ENABLE);
                        log_fusa("ECC_SB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_SB_STATUS, ECC_SB_STAT);
                        log_fusa("ECC_DB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_DB_STATUS, ECC_DB_STAT);
                }
                break;
        case FUNC_SAFETY_ITEM_SM8:
        case FUNC_SAFETY_ITEM_SM9:
        case FUNC_SAFETY_ITEM_SM10:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                fmcw_radio_sm_saturation_detector_fault_injection_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                break;
        case FUNC_SAFETY_ITEM_SM11:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM11 IF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm11_runtime = %6.4f\n", time_calc_t12);
                log_fusa("SM11 IF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);*/
                break;
        case FUNC_SAFETY_ITEM_SM12:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM12 RF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm12_runtime = %6.4f\n", time_calc_t12);
                log_fusa("SM12 RF Loopback IRQ = %d\n", IRQ_value);
                if (IRQ_value)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);*/
                break;
        case FUNC_SAFETY_ITEM_SM13:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_chirp_monitor_fault_injection_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM13 Chirp Monitor IRQ = %d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm13_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM14:
                //fault injection
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio, true);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                log_fusa("SM14 Over Temp Detector IRQ = %d\n", IRQ_value);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio,false);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm14_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM201:
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                ret = func_safety_sm_vga_consistence_check();
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                if (ret == true)
                        func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                func_safety_sm_vga_consistence_check();
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm201_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM101:
                //disable sm6
                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, false);

                baseband_bist_ctrl(cfg->bb, false);

                //clear sm6 err status and enable sm6
                //value = raw_read_op(REG_EMU_RF_ERR_STA);
                //log_fusa("RF_ERR_STA after: 0x%x, value: 0x%x.\n", REG_EMU_RF_ERR_STA, value);
                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, true);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                baseband_bist_ctrl(cfg->bb, false);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm101_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM102:
                //set CPU.ERP_CTRL to 0x00 to disable cpu ecc
                core_reg_write(REG_CPU_ERP_CTRL, 0x00);
                log_fusa("REG_CPU_ERP_CTRL_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, 0x00);
                value = core_reg_read(REG_CPU_ERP_CTRL);
                log_fusa("REG_CPU_ERP_CTRL_ADDR_READ: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, value);

                //write data 0x55AA66BB to address 0xA07FFC
                raw_write_op(0x107FFC, 0x55AA66BB);
                log_fusa("WRITE_DATA_TO_0xA07FFC_ADDR: 0x%x, value: 0x%x.\n", 0x107FFC, 0x55AA66BB);

                //set CPU.ERP_CTRL to 0xf to enable cpu ecc
                core_reg_write(REG_CPU_ERP_CTRL, 0xf);
                log_fusa("REG_CPU_ERP_CTRL_ADDR_WRITE: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, 0xf);
                value = core_reg_read(REG_CPU_ERP_CTRL);
                log_fusa("REG_CPU_ERP_CTRL_ADDR_READ: 0x%x, value: 0x%x.\n", REG_CPU_ERP_CTRL, value);

                //read address 0xA07FFC data
                value = raw_read_op(0x107FFC);
                log_fusa("REG_0xA07FFC_ADDR_READ: 0x%x, value: 0x%x.\n", 0x107FFC, value);

                //wait fault
                break;
        case FUNC_SAFETY_ITEM_SM110:
                func_safety_crc_acceleration(func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM113:
                arc_wdt_on_flag = false;
                //enable cpu watchdog timer, period and event_timeout
                break;
        case FUNC_SAFETY_ITEM_SM120:
                //can_loopback_test_flag = true;
                func_safety_sm_can_loopback_init();
                //func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_can_loopback_init();
                //gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm120_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM121:
        case FUNC_SAFETY_ITEM_SM122:
        case FUNC_SAFETY_ITEM_SM123:
        case FUNC_SAFETY_ITEM_SM124:
        case FUNC_SAFETY_ITEM_SM125:
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM126:
                func_safety_sm_spi_loopback(func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_spi_loopback(func_safety_error_type);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm126_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM128:
                fusa_i2c_ack_write(0x50, 0x00, fusa_test_wr_buf, 8, func_safety_error_type, func_safety_sm_num);
                //func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM129:
                func_safety_sm_can_config_reg_protection(func_safety_error_type);
                //func_safety_error_handler(func_safety_sm_num, func_safety_error_type);

                //runtime test
                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_can_config_reg_protection(func_safety_error_type);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm129_runtime = %6.4f\n", time_calc_t12);*/
                break;
        case FUNC_SAFETY_ITEM_SM130:
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM133:
                func_safety_error_handler(func_safety_sm_num, func_safety_error_type);
                break;
        case FUNC_SAFETY_ITEM_SM202:
                fusa_i2c_Periodic_Software_readback(0x50, 0x00, fusa_test_wr_buf, 1, func_safety_error_type, func_safety_sm_num);
                break;
        case FUNC_SAFETY_ITEM_SM203:
                fusa_i2c_readback_write(0x50, 0x00, fusa_test_wr_buf, 8, func_safety_error_type, func_safety_sm_num);
                break;
        case FUNC_SAFETY_ITEM_SM204:
                fusa_i2c_crc_write(0x50, 0x00, fusa_test_wr_buf, 8, func_safety_error_type, func_safety_sm_num);
                fusa_i2c_crc_read(0x50, 0x00, fusa_test_rd_buf, 8, func_safety_error_type, func_safety_sm_num);
                break;
        case FUNC_SAFETY_ITEM_SM205:
                fusa_on_time_error_pin_check();
                break;
        case FUNC_SAFETY_ITEM_SM206:
                external_wdt_test_flag = true;
                break;
        case FUNC_SAFETY_ITEM_SM805:
                func_safety_sm_periodic_readback_configuration_registers(func_safety_error_type);

                /*time_calc_t1 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                gpio_write(LED_D1_NO, LED_D1_ON);
                func_safety_sm_periodic_readback_configuration_registers(func_safety_error_type);
                gpio_write(LED_D1_NO, LED_D1_OFF);
                time_calc_t2 = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS;
                time_calc_t12 = time_calc_t2 - time_calc_t1;
                log_fusa("\t sm805_runtime = %6.4f\n", time_calc_t12);*/
                break;
        default:
                log_fusa("\n");
                break;
        }
#endif
}

/****************************************************
 * NAME         : func_safety_sm_vga_consistence_check
 * DESCRIPTIONS : sm201, vga consistence check test handler
 * INPUT        : None
 * OUTPUT       : true or false
 *****************************************************/
static bool func_safety_sm_vga_consistence_check(void)
{
        bool ret = false;
        baseband_t* bb = baseband_get_rtl_frame_type(); /* read back the bank selected in RTL */
        // sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(&bb->radio, baseband_t, radio)->cfg;
        //sensor_config_t* cfg = (sensor_config_t*)(bb->cfg);
        baseband_hw_t* bb_hw = &bb->bb_hw;

        //gpio_write(LED_D1_NO, LED_D1_ON);//output low level

        // baseband dump init
        uint32_t old_buf_store = baseband_switch_buf_store(bb_hw,SYS_BUF_STORE_ADC);
        uint8_t old_bnk = BB_READ_REG(bb_hw, FDB_SYS_BNK_ACT); /* read back the bank selected in RTl */
        uint16_t old_status_en = BB_READ_REG(bb_hw, SYS_ENABLE);

        /* set chirp number and phase scramble init state*/
        fmcw_radio_switch_bank(&bb->radio, 5 + bb->radio.frame_type_id);
        uint32_t old_chirp_size_0 = fmcw_radio_reg_read(&bb->radio, R5_FMCW_CHIRP_SIZE_1_0);
        uint32_t old_chirp_size_1 = fmcw_radio_reg_read(&bb->radio, R5_FMCW_CHIRP_SIZE_1_1);
        uint32_t old_bb_sys_size_vel = BB_READ_REG(bb_hw, SYS_SIZE_VEL_BUF);
        uint32_t old_bb_sys_size_fft = BB_READ_REG(bb_hw, SYS_SIZE_VEL_FFT);
        uint32_t old_bb_sys_bpm = BB_READ_REG(bb_hw, SYS_SIZE_BPM);

        fmcw_radio_reg_write(&bb->radio, R5_FMCW_CHIRP_SIZE_1_0, 1);
        fmcw_radio_reg_write(&bb->radio, R5_FMCW_CHIRP_SIZE_1_1, 0);
        BB_WRITE_REG(bb_hw, SYS_SIZE_VEL_BUF, 0);
        BB_WRITE_REG(bb_hw, SYS_SIZE_VEL_FFT, 0);
        BB_WRITE_REG(bb_hw, SYS_SIZE_BPM, 0);
        /* turn off zero doppler removel */
        //bool old_zer_dpl = BB_READ_REG(bb_hw, FFT_ZER_DPL_ENB);
        //BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB  , 0);
        //dmu_adc_reset(); // ADC should reset in cascade
        //close all tx
        fmcw_radio_tx_ch_on(&bb->radio, -1, false);

        // log_fusa("consistence check start time = %f\n", fusa_get_current_time_ms());
        /* start baseband */
        baseband_start_with_params(bb, true, false,
                        (1 << SYS_ENABLE_SAM_SHIFT), false, false,
                        false);
        /* wait done */
        BASEBAND_ASSERT(baseband_wait_bb_done(POL_MODE, DEFAULT_TIMOUT) == E_OK);
        BB_WRITE_REG(NULL, SYS_IRQ_CLR, BB_IRQ_CLEAR_BB_DONE);
        // log_fusa("consistence check end time = %f\n", fusa_get_current_time_ms());
        // Search test target peak in 2D-FFT plane
        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
        //gpio_write(LED_D1_NO, LED_D1_OFF);//output low level

        uint32_t fft_mem;
        int ch_index = 0;
        int tmp_rng_ind = 0;
        int rng_ind_end = (BB_READ_REG(bb_hw, SYS_SIZE_RNG_FFT) + 1) / 2;
        int count = 0;
        //int16_t *BufPtr;
        int16_t temp_value1, temp_value2;
        int32_t rx_sum_src_value_buf[4] = { 0, 0, 0, 0 };
        float rx_average_src_value_buf[4] = { 0, 0, 0, 0 };
        uint32_t rx_sum_square_value_buf[4] = { 0, 0, 0, 0 };
        float rx_sum_diff[4] = { 0, 0, 0, 0 };

        for (ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {

                count = 0;

                //for (int j = 0; j < 10; j++) {
                tmp_rng_ind = 0;
                rng_ind_end = 200;
                for (; tmp_rng_ind < rng_ind_end; tmp_rng_ind++) {
                        fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index,
                                        tmp_rng_ind, 0, 0);
                        temp_value1 = ((int16_t) ((fft_mem >> 16) & 0xFFFF)) / 4;
                        temp_value2 = ((int16_t) (fft_mem & 0xFFFF)) / 4;
                        //log_fusa("%d ", temp_value1); // Print int16_t to UART
                        //log_fusa("%d ", temp_value2); // Print int16_t  to UART

                        rx_sum_src_value_buf[ch_index] +=
                                        (temp_value1 + temp_value2);
                        rx_sum_square_value_buf[ch_index] += (temp_value1
                                        * temp_value1 + temp_value2 * temp_value2);

                        count++;
                        //gpio_write(LED_D1_NO, LED_D1_OFF);
                }
                //}
                //log_fusa("\n");
                rx_average_src_value_buf[ch_index] =
                                rx_sum_src_value_buf[ch_index] / 512.0;
        }
        //log_fusa("one frame done!\n");
        //log_fusa("rx_sum_src_value_buf:  %d  %d  %d  %d  \n", rx_sum_src_value_buf[0], rx_sum_src_value_buf[1], rx_sum_src_value_buf[2], rx_sum_src_value_buf[3]);
        //log_fusa("rx_average_src_value_buf:  %.4f  %.4f  %.4f  %.4f  \n", rx_average_src_value_buf[0], rx_average_src_value_buf[1], rx_average_src_value_buf[2], rx_average_src_value_buf[3]);
        //log_fusa("rx_sum_square_value_buf:  %d  %d  %d  %d  \n", rx_sum_square_value_buf[0], rx_sum_square_value_buf[1], rx_sum_square_value_buf[2], rx_sum_square_value_buf[3]);

        for (int k = 0; k < MAX_NUM_RX; k++) {
                rx_sum_diff[k] = (rx_sum_square_value_buf[k]
                                - 512 * rx_average_src_value_buf[k]
                                                * rx_average_src_value_buf[k]);
        }
        //log_fusa("rx_sum_diff:  %.4f  %.4f  %.4f  %.4f  \n", rx_sum_diff[0], rx_sum_diff[1], rx_sum_diff[2], rx_sum_diff[3]);

        //gpio_write(LED_D1_NO, LED_D1_OFF);//output low level
        //log_fusa("\r\n");
        float logrx[4], diff[4], avg;

        logrx[0] = 10 * log10f(rx_sum_diff[0]);
        logrx[1] = 10 * log10f(rx_sum_diff[1]);
        logrx[2] = 10 * log10f(rx_sum_diff[2]);
        logrx[3] = 10 * log10f(rx_sum_diff[3]);

        // avg = (logrx[0] + logrx[1] + logrx[2] + logrx[3])/4;
        avg = 10 * log10f(rx_sum_diff[0]/4 + rx_sum_diff[1]/4 + rx_sum_diff[2]/4 + rx_sum_diff[3]/4);

        diff[0] = fabs(logrx[0] - avg);
        diff[1] = fabs(logrx[1] - avg);
        diff[2] = fabs(logrx[2] - avg);
        diff[3] = fabs(logrx[3] - avg);
        // restore tx parameter
        fmcw_radio_tx_restore(&bb->radio);

        if ((diff[0] > VGA_DIFF_VALUE) || \
                (diff[1] > VGA_DIFF_VALUE) || \
                (diff[2] > VGA_DIFF_VALUE) || \
                (diff[3] > VGA_DIFF_VALUE)) {
                ret = true;
                log_fusa("consistence_check result, diff0: %.2f diff1: %.2f diff2: %.2f diff3: %.2f avg = %f; ret = %d\n",diff[0], diff[1], diff[2], diff[3], avg, ret);
        } else {
                ret = false;
        }
        baseband_switch_mem_access(bb_hw, old);

        /* restore zero doppler removel */
        //BB_WRITE_REG(bb_hw, FFT_ZER_DPL_ENB, old_zer_dpl);
        BB_WRITE_REG(bb_hw, SYS_BNK_ACT, old_bnk);
        BB_WRITE_REG(bb_hw, SYS_ENABLE, old_status_en);
        baseband_switch_buf_store(bb_hw, old_buf_store);
        /* set chirp number and phase scramble init state*/
        fmcw_radio_switch_bank(&bb->radio, 5 + bb->radio.frame_type_id);
        fmcw_radio_reg_write(&bb->radio, R5_FMCW_CHIRP_SIZE_1_0, old_chirp_size_0);
        fmcw_radio_reg_write(&bb->radio, R5_FMCW_CHIRP_SIZE_1_1, old_chirp_size_1);
        BB_WRITE_REG(bb_hw, SYS_SIZE_VEL_BUF, old_bb_sys_size_vel);
        BB_WRITE_REG(bb_hw, SYS_SIZE_VEL_FFT, old_bb_sys_size_fft);
        BB_WRITE_REG(bb_hw, SYS_SIZE_BPM, old_bb_sys_bpm);
        BB_WRITE_REG(NULL, SYS_IRQ_CLR, BB_IRQ_CLEAR_ALL);
        return ret;
}

/****************************************************
 * NAME         : func_safety_sm_can_config_reg_protection
 * DESCRIPTIONS : sm129, can config re protection handler
 * INPUT        : func_safety_error_type: error type ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_can_config_reg_protection(uint8_t func_safety_error_type)
{
        uint32_t value1[9];
        uint32_t value2[9];
        uint8_t i;
        bool flag = false;

        value1[0] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);    //MCR
        //log_fusa("MCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value1[0]);
        value1[1] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET); //PCR
        //log_fusa("PCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET, value1[1]);
        value1[2] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET);    //TSCCR
        //log_fusa("TSCCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET, value1[2]);
        value1[3] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET); //DBTCR
        //log_fusa("DBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET, value1[3]);
        value1[4] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_NML_BIT_TIME_CTRL_OFFSET);    //NBTCR
        //log_fusa("NBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_NML_BIT_TIME_CTRL_OFFSET, value1[4]);
        value1[5] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET); //IDFCR
        //log_fusa("IDFCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET, value1[5]);
        value1[6] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET); //XIDAMR
        //log_fusa("XIDAMR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET, value1[6]);
        value1[7] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_RX_EL_SIZE_CFG_OFFSET);     //RXESCR
        //log_fusa("RXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_RX_EL_SIZE_CFG_OFFSET, value1[7]);
        value1[8] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_TX_EL_SIZE_CFG_OFFSET);     //TXESCR
        //log_fusa("TXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TX_EL_SIZE_CFG_OFFSET, value1[8]);

        //log_fusa("write 0xAAAAAAAA to MCR PCR TSCCR DBTCR NBTCR IDFCR XIDAMR RXESCR TXESCR.\n");
        //raw_write_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, 0xAAAAAAAA);//MCR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET, 0xAAAAAAAA); //PCR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET,
                        0xAAAAAAAA);        //TSCCR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET,
                        0xAAAAAAAA);        //DBTCR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_NML_BIT_TIME_CTRL_OFFSET,
                        0xAAAAAAAA);        //NBTCR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET, 0xAAAAAAAA); //IDFCR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET,
                        0xAAAAAAAA);        //XIDAMR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_RX_EL_SIZE_CFG_OFFSET,
                        0xAAAAAAAA);        //RXESCR
        raw_write_op(REL_REGBASE_CAN0 + REG_CAN_TX_EL_SIZE_CFG_OFFSET,
                        0xAAAAAAAA);        //TXESCR

        value2[0] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET);    //MCR
        //log_fusa("MCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_MODE_CTRL_OFFSET, value2[0]);
        value2[1] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET); //PCR
        //log_fusa("PCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_PROTOCOL_CTRL_OFFSET, value2[1]);
        value2[2] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET);    //TSCCR
        //log_fusa("TSCCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TIMESTAMP_COUNTER_CFG_OFFSET, value2[2]);
        value2[3] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET); //DBTCR
        //log_fusa("DBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_DATA_BIT_TIME_CTRL_OFFSET, value2[3]);
        value2[4] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_NML_BIT_TIME_CTRL_OFFSET);    //NBTCR
        //log_fusa("NBTCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_NML_BIT_TIME_CTRL_OFFSET, value2[4]);
        value2[5] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET); //IDFCR
        //log_fusa("IDFCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_ID_FILTER_CTRL_OFFSET, value2[5]);
        value2[6] = raw_read_op(REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET); //XIDAMR
        //log_fusa("XIDAMR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_EXTEND_ID_AND_MASK_OFFSET, value2[6]);
        value2[7] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_RX_EL_SIZE_CFG_OFFSET);     //RXESCR
        //log_fusa("RXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_RX_EL_SIZE_CFG_OFFSET, value2[7]);
        value2[8] = raw_read_op(
        REL_REGBASE_CAN0 + REG_CAN_TX_EL_SIZE_CFG_OFFSET);     //TXESCR
        //log_fusa("TXESCR_ADDR_READ: 0x%x, value: 0x%x.\n", REL_REGBASE_CAN0 + REG_CAN_TX_EL_SIZE_CFG_OFFSET, value2[8]);

        for (i = 0; i < 9; i++) {
                if (value1[i] != value2[i]) {
                        flag = true;
                        break;
                }
        }

        if(flag == true){
                func_safety_error_handler(fusaConfList[SM_INDEX_28].sm_num, fusaConfList[SM_INDEX_28].error_type);
        } else {
                //log_fusa("func_safety_sm_can_config_reg_protection test ok.\n");
        }
}

/****************************************************
 * NAME         : func_safety_sm_periodic_readback_configuration_registers
 * DESCRIPTIONS : sm805, periodic software readback of configuration registers
 * INPUT        : func_safety_error_type: error type IRQ or ss1 or ss2
 * OUTPUT       : None
 *****************************************************/
static void func_safety_sm_periodic_readback_configuration_registers(uint8_t func_safety_error_type)
{
        uint32_t reg_value[64];
        uint8_t i;
        static uint8_t act_flag = 0x00;
        static uint32_t reg_value_crc_surce,reg_value_crc_curret;

        //read Clock and Reset Register
        reg_value[0] = raw_read_op(REG_CLKGEN_READY_50M);
        reg_value[1] = raw_read_op(REG_CLKGEN_READY_PLL);
        reg_value[2] = raw_read_op(REG_CLKGEN_SEL_300M);
        reg_value[3] = raw_read_op(REG_CLKGEN_SEL_400M);
        reg_value[4] = raw_read_op(REG_CLKGEN_DIV_CPU);
        reg_value[5] = raw_read_op(REG_CLKGEN_DIV_MEM);
        reg_value[6] = raw_read_op(REG_CLKGEN_DIV_AHB);
        reg_value[7] = raw_read_op(REG_CLKGEN_DIV_APB);
        reg_value[8] = raw_read_op(REG_CLKGEN_ENA_ROM);
        reg_value[9] = raw_read_op(REG_CLKGEN_ENA_RAM);
        reg_value[10] = raw_read_op(REG_CLKGEN_ENA_BB_TOP);
        reg_value[11] = raw_read_op(REG_CLKGEN_ENA_BB_CORE);
        reg_value[12] = raw_read_op(REG_CLKGEN_ENA_FLASH_CTRL);
        reg_value[13] = raw_read_op(REG_CLKGEN_ENA_DMU);
        reg_value[14] = raw_read_op(REG_CLKGEN_RSTN_BB_TOP);
        reg_value[15] = raw_read_op(REG_CLKGEN_RSTN_BB_CORE);
        reg_value[16] = raw_read_op(REG_CLKGEN_RSTN_FLASH_CTRL);
        reg_value[17] = raw_read_op(REG_CLKGEN_RSTN_DMU);

        //read DMU Registers
        reg_value[18] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_QSPI_OFFSET);
        reg_value[19] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M1_OFFSET);
        reg_value[20] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_UART0_OFFSET);
        reg_value[21] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_UART1_OFFSET);
        reg_value[22] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_CAN0_OFFSET);
        reg_value[23] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_CAN1_OFFSET);
        reg_value[24] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_RESET_OFFSET);
        reg_value[25] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SYNC_OFFSET);
        reg_value[26] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_I2C_OFFSET);
        reg_value[27] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_PWM0_OFFSET);
        reg_value[28] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_PWM1_OFFSET);
        reg_value[29] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_ADC_CLK_OFFSET);
        reg_value[30] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_CAN_CLK_OFFSET);
        reg_value[31] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_M0_OFFSET);
        reg_value[32] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S_OFFSET);
        reg_value[33] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_CLK_OFFSET);
        reg_value[34] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_SEL_OFFSET);
        reg_value[35] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MOSI_OFFSET);
        reg_value[36] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_SPI_S1_MISO_OFFSET);
        reg_value[37] = raw_read_op(REL_REGBASE_DMU + REG_DMU_MUX_JTAG_OFFSET);
        reg_value[38] = raw_read_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET);
        reg_value[39] = raw_read_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMINI_ENA_OFFSET);
        reg_value[40] = raw_read_op(REL_REGBASE_DMU + REG_DMU_SYS_OTP_PRGM_EN_OFFSET);
        reg_value[41] = raw_read_op(REL_REGBASE_DMU + REG_DMU_SYS_OTP_ECC_EN_OFFSET);

        //read EMU Registers
        reg_value[42] = raw_read_op(REG_EMU_SEC_STA);
        reg_value[43] = raw_read_op(REG_EMU_SEC_CTRL);
        reg_value[44] = raw_read_op(REG_EMU_SAFETY_PAD_CTRL);
        reg_value[45] = raw_read_op(REG_EMU_FSM_STA);
        reg_value[46] = raw_read_op(REG_EMU_RF_ERR_IRQ_ENA);
        reg_value[47] = raw_read_op(REG_EMU_RF_ERR_IRQ_MASK);
        reg_value[48] = raw_read_op(REG_EMU_RF_ERR_SS1_ENA);
        reg_value[49] = raw_read_op(REG_EMU_RF_ERR_SS1_MASK);
        reg_value[50] = raw_read_op(REG_EMU_RF_ERR_SS2_ENA);
        reg_value[51] = raw_read_op(REG_EMU_RF_ERR_SS2_MASK);
        reg_value[52] = raw_read_op(REG_EMU_RF_TEST_DIV);
        reg_value[53] = raw_read_op(REG_EMU_DIG_ERR_IRQ_ENA);
        reg_value[54] = raw_read_op(REG_EMU_DIG_ERR_IRQ_MASK);
        reg_value[55] = raw_read_op(REG_EMU_DIG_ERR_SS1_ENA);
        reg_value[56] = raw_read_op(REG_EMU_DIG_ERR_SS1_MASK);
        reg_value[57] = raw_read_op(REG_EMU_DIG_ERR_SS2_ENA);
        reg_value[58] = raw_read_op(REG_EMU_DIG_ERR_SS2_MASK);
        reg_value[59] = raw_read_op(REG_EMU_DIG_TEST_DIV);
        reg_value[60] = raw_read_op(REG_EMU_LBIST_STA);
        reg_value[61] = raw_read_op(REG_EMU_REBOOT_CNT);
        reg_value[62] = raw_read_op(REG_EMU_REBOOT_LIMIT);

        //read RF registers
        uint8_t old_bank = fmcw_radio_switch_bank(NULL, 0);
        reg_value[63] = RADIO_READ_BANK_REG(0, CBC_EN);
        reg_value[63] &= 0x07;
        fmcw_radio_switch_bank(NULL, old_bank);
        /*log_fusa("\r\n");
        for (i = 0; i < 64; i++) {
                log_fusa("0x%x \n", reg_value[i]);
        }
        log_fusa("\r\n");*/

        if (act_flag == 0) {
                reg_value_crc_surce = 0x00000000;
                for (i = 0; i < 64; i++) {
                        reg_value_crc_surce += reg_value[i];
                }
                act_flag = 0xff;
                reg_value_crc_curret = reg_value_crc_surce;
        } else {
                reg_value_crc_curret = 0x00000000;
                for (i = 0; i < 64; i++) {
                        reg_value_crc_curret += reg_value[i];
                }
        }

        if(reg_value_crc_curret == reg_value_crc_surce) {
                //log_fusa("SM805 func_safety_sm_periodic_readback_configuration_registers test ok.\n");
        } else {
                func_safety_error_handler(fusaConfList[SM_INDEX_33].sm_num, fusaConfList[SM_INDEX_33].error_type);
        }
}



/* Start Baseband once to implement read/write process on Baseband memory */
/* Supposedly, better this to be done by hardware */
void clear_bb_memory_by_scan_start_bb_one_time(void)
{
        uint16_t bb_status_en = 0;
        baseband_t* bb = baseband_get_rtl_frame_type(); /* read back the bank selected in RTL */

        bb_status_en = ((SYS_ENABLE_SAM_MASK << SYS_ENABLE_SAM_SHIFT)
                      | (SYS_ENABLE_FFT_2D_MASK << SYS_ENABLE_FFT_2D_SHIFT)
                      | (SYS_ENABLE_CFR_MASK << SYS_ENABLE_CFR_SHIFT)
                      | (SYS_ENABLE_BFM_MASK << SYS_ENABLE_BFM_SHIFT));

        /* Clear event bit before bb start */
        uint32_t event_bits = baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
        if( event_bits != E_OK)
        {
                log_fusa("Event bit set to %d before start @%s", event_bits, __func__);
        }

        /* start baseband */
        baseband_start_with_params(bb, true, true, bb_status_en, true, BB_IRQ_DISABLE_ALL, false);

        /* wait done */
        BASEBAND_ASSERT(baseband_wait_bb_done(POL_MODE, DEFAULT_TIMOUT) == E_OK);

        log_fusa("/*** clear_bb_memory_by_scan_start_bb_one_time:done! ***/\n\r");

        /* stop baseband */
        baseband_stop(bb);
        BB_WRITE_REG(NULL, SYS_IRQ_CLR, BB_IRQ_CLEAR_ALL);
}

void clear_bb_memory_ecc_status_bit(void)
{
        //uint32_t value_sb,value_db;

        if (fusaConfList[SM_INDEX_12].open_flag == true) {
                /*value_sb = raw_read_op(BB_REG_SYS_ECC_SB_STATUS);
                log_fusa("ECC_SB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_SB_STATUS, value_sb);
                value_db =  raw_read_op(BB_REG_SYS_ECC_DB_STATUS);
                log_fusa("ECC_DB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_DB_STATUS, value_db);*/

                baseband_write_reg(NULL, BB_REG_SYS_ECC_SB_CLR, 0x2000);
                baseband_write_reg(NULL, BB_REG_SYS_ECC_DB_CLR, 0x2000);

                /*value_sb = raw_read_op(BB_REG_SYS_ECC_SB_STATUS);
                log_fusa("ECC_SB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_SB_STATUS, value_sb);
                value_db =  raw_read_op(BB_REG_SYS_ECC_DB_STATUS);
                log_fusa("ECC_DB_STAT_ADDR: 0x%x, value: 0x%x.\n", BB_REG_SYS_ECC_DB_STATUS, value_db);*/
        }
}

void safety_mechanism_power_on_check(fmcw_radio_t *radio)
{
        log_fusa("FEI start!\r\n"); //just for autotest use
#if SAFETY_MECHANISM_POWER_ON_CHECK_EN
        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);
        uint32_t IRQ_value;
        uint32_t IRQ_all = 0x1;

#if (SAFETY_FEATURE_SM901 == FEATURE_ON)
        /* sm_ldo_monitor fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_ldo_monitor_fault_injection_IRQ(radio);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm LDO Monitor IRQ = %d\n",IRQ_value);
#endif
#if ((SAFETY_FEATURE_SM902 == FEATURE_ON) && (SAFETY_FEATURE_SM2 == FEATURE_ON))
        /* sm_avdd33_monitor fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_avdd33_monitor_IRQ(radio,true);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm AVDD33 Monitor IRQ = %d\n",IRQ_value);
#endif
#if ((SAFETY_FEATURE_SM902 == FEATURE_ON) && (SAFETY_FEATURE_SM3 == FEATURE_ON))
        /* sm_dvdd11_monitor fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_dvdd11_monitor_IRQ(radio,true);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm DVDD11 Monitor IRQ = %d\n",IRQ_value);
#endif
#if (SAFETY_FEATURE_SM904 == FEATURE_ON)
        /* sm_bg_monitor fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_bg_monitor_IRQ(radio,true);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm BG Monitor IRQ = %d\n",IRQ_value);
#endif
#if (SAFETY_FEATURE_SM905 == FEATURE_ON)
        /* sm_rfpower_detector fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_rfpower_detector_fault_injection_IRQ(radio,5);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm RF Power Detector IRQ = %d\n",IRQ_value);
#endif
#if (SAFETY_FEATURE_SM906 == FEATURE_ON)
        /* sm_saturation_detector fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_saturation_detector_fault_injection_IRQ(radio);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm Saturation Detector IRQ = 0x%x\n",IRQ_value);
#endif
#if (SAFETY_FEATURE_SM907 == FEATURE_ON)
        /* sm_if_loopback fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_if_loopback_IRQ(radio,true);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm IF Loopback IRQ = %d\n",IRQ_value);
#endif
#if (SAFETY_FEATURE_SM908 == FEATURE_ON)
        /* sm_rf_loopback fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(radio,true);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm RF Loopback IRQ = %d\n",IRQ_value);
#endif
#if (SAFETY_FEATURE_SM910 == FEATURE_ON)
        /* sm_over_temp_detector fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        IRQ_value = fmcw_radio_sm_over_temp_detector_IRQ(radio,true);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm Over Temp Detector IRQ = %d\n",IRQ_value);
#endif
#if (SAFETY_FEATURE_SM911 == FEATURE_ON)
        /* sm_chirp_monitor fault injection */
        log_fusa("time_start = %f\n",fusa_get_current_time_ms());
        int32_t flag = int_enabled(INT_RF_ERROR_IRQ);
        if(flag)
        {
                int_disable(INT_RF_ERROR_IRQ);
        }
        IRQ_value = fmcw_radio_sm_chirp_monitor_fault_injection_IRQ(radio);
        IRQ_all = IRQ_all && IRQ_value;
        log_fusa("time_end = %f\n",fusa_get_current_time_ms());
        log_fusa("sm Chirp Monitor IRQ = %d\n",IRQ_value);
        if(flag)
        {
                int_enable(INT_RF_ERROR_IRQ);
        }
#endif
        fmcw_radio_switch_bank(radio, old_bank);
        if (IRQ_all)
        {
                log_fusa("/*** sm self-check done: IRQ_all = %d  success!***/\n\r", IRQ_all);
        }
        else
        {
                log_fusa("/*** sm self-check done: IRQ_all = %d  failed!***/\n\r", IRQ_all);
        }
#endif
        log_fusa("FEI end!\r\n"); //just for autotest use
}

/****************************************************
 * NAME         : get_can_send_status
 * DESCRIPTIONS : get can intr send status
 * INPUT        : None
 * OUTPUT       : CAN_SEND_STATUS_IDLE: can idle.
 *                CAN_SEND_STATUS_SENDING: can intr send all data finish
 *****************************************************/
uint8_t get_can_send_status(void)
{
        return can_send_status;
}

/****************************************************
 * NAME         : set_can_send_status
 * DESCRIPTIONS : set can intr send status
 * INPUT        : CAN_SEND_STATUS_IDLE: can idle.
 *                CAN_SEND_STATUS_SENDING: can intr send all data finish
 * OUTPUT       : None
 *****************************************************/
void set_can_send_status(uint8_t value)
{
        can_send_status = value;
}

/****************************************************
 * NAME         : func_safety_sm_ldo_monitor_set_part
 * DESCRIPTIONS : sm1 set part handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_ldo_monitor_set_part(fmcw_radio_t *radio)
{
        if (fusaConfList[SM_INDEX_0].open_flag == true) {
                if (bb_frame_start_flag == true) {
                        bb_frame_start_flag = false;
                        ldo_set_part_flag = true;
                        // log_fusa("set part %d\n",sm1_ldo_part_cnt);
                        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_0_SHIFT);
                        fmcw_radio_sm_ldo_monitor_setting(radio, sm1_ldo_part_cnt);
                }
        }
}

/****************************************************
 * NAME         : func_safety_sm_ldo_monitor_read_part
 * DESCRIPTIONS : sm1 read part handler
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_ldo_monitor_read_part(void)
{
        static uint8_t ldo_read_part_time_cnt = 0x00;
        baseband_t *bb = baseband_get_cur_bb();

        if (fusaConfList[SM_INDEX_0].open_flag == true) {
                if (ldo_set_part_flag == true) {
                        ldo_read_part_time_cnt++;
                        if (ldo_read_part_time_cnt > 18) {
                                ldo_read_part_time_cnt = 0x00;
                                ldo_set_part_flag = false;
                                // log_fusa("read_part %d\n",sm1_ldo_part_cnt);
                                fmcw_radio_sm_ldo_monitor_IRQ(&bb->radio);
                                sm1_ldo_part_cnt++;
                                if (sm1_ldo_part_cnt > 3){
                                        sm1_ldo_part_cnt = 0x00;
                                }
                        }
                }
        }
}

/****************************************************
 * NAME         : func_safety_sm_periodic_run_handler
 * DESCRIPTIONS : periodic sm run handler: SM120 SM126 SM1
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_sm_periodic_run_handler(void)
{
        if (get_track_cfg() == CAN_INT) {
                if ((get_can_send_status() == CAN_SEND_STATUS_IDLE) && (periodic_sm_finish_flag == true)) {
                        periodic_sm_finish_flag = false;

                        if (fusaConfList[SM_INDEX_21].open_flag == true) {
                                func_safety_sm_can_loopback_init();
                        }

                        if (fusaConfList[SM_INDEX_27].open_flag == true) {
                                func_safety_sm_spi_loopback(fusaConfList[SM_INDEX_27].error_type);
                        }
                }
        }
#if SAFETY_FEATURE_SM1 == FEATURE_ON
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM1, FUNC_SAFETY_ITEM_SM1_READ_PART);
#endif
        // func_safety_sm_ldo_monitor_read_part();
}

/****************************************************
 * NAME         : func_safety_enable_can_ecc
 * DESCRIPTIONS : enable CAN ECC
 * INPUT        : id: can id
 * OUTPUT       : None
 *****************************************************/
void func_safety_enable_can_ecc(uint32_t id)
{
        uint32_t value;
        uint32_t addr_base;
        if (fusaConfList[SM_INDEX_20].open_flag == true) {

                if (id == CAN_0_ID)
                {
                        addr_base = REL_REGBASE_CAN0;
                }
                else if(id == CAN_1_ID)
                {
                        addr_base = REL_REGBASE_CAN1;
                }
                else
                {
                        return;
                }
                //set CAN_0.MCR.CFG to 0x1
                value = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
                raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_CFG);
                //set CAN_0.MCR.ECCENA to 0x1
                value = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
                raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, value | BIT_MODE_CTRL_ECCENA);
                // set CAN_0.IE. set CAN_0.IE.BEUE and CAN_0.IE.BECE to 0x1
                value = raw_read_op(addr_base + REG_CAN_ISR_ENABLE_OFFSET);
                raw_write_op(addr_base + REG_CAN_ISR_ENABLE_OFFSET, value | BIT_INTERRUPT_BEU | BIT_INTERRUPT_BEC);
                //set CAN_0.ILS0R.BEULS and CAN_0.ILS0R.BECLS to 0x3
                value = raw_read_op(addr_base + REG_CAN_ISR_LINE_SELECT0_OFFSET);
                raw_write_op(addr_base + REG_CAN_ISR_LINE_SELECT0_OFFSET,
                    value |
                    (BITS_ISR_LINE_SELECT_ELOLS_MASK << BITS_ISR_LINE_SELECT_BEULS_SHIFT) |
                    (BITS_ISR_LINE_SELECT_BEULS_MASK << BITS_ISR_LINE_SELECT_BECLS_SHIFT));
                //clear CAN0.IR register,write 1 clear
                value = raw_read_op(addr_base + REG_CAN_ISR_OFFSET);
                raw_write_op(addr_base + REG_CAN_ISR_OFFSET, value | BIT_INTERRUPT_BEC | BIT_INTERRUPT_BEU);
                //set CAN_0.ILE.EINT3 to 0x1
                value = raw_read_op(addr_base + REG_CAN_ISR_LINE_ENABLE_OFFSET);
                raw_write_op(addr_base + REG_CAN_ISR_LINE_ENABLE_OFFSET, value | BIT_ISR_LINE_ENABLE_EINT3);

                //set CAN_0.MCR.CFG to 0x0
                value = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
                raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, value & (~BIT_MODE_CTRL_CFG));

        }
}

/****************************************************
 * NAME         : func_safety_process
 * DESCRIPTIONS : used to run the items periodically
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void func_safety_process(void *params)
{
        uint32_t IRQ_value = 0;
        bool test_result;
        baseband_t *bb = baseband_get_cur_bb();
        sensor_config_t *cfg = sensor_config_get_cur_cfg();

        periodic_sm_finish_flag = false;

        switch(FUNC_SAFETY_PERIODIC_SM_CLCLE) {
        case PERIODIC_SM_1_CYCLE:
                clear_bb_memory_ecc_status_bit();
                /* SM6 and SM201 */
                if ((fusaConfList[SM_INDEX_5].open_flag == true) || (fusaConfList[SM_INDEX_11].open_flag == true)) {
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                        if (fusaConfList[SM_INDEX_5].open_flag == true)
                        {
                                // log_fusa("rfpower_detector_start = %f\n",fusa_get_current_time_ms());
                                func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_5_SHIFT);
                                IRQ_value = fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
                                // log_fusa("rfpower_detector_end = %f\n",fusa_get_current_time_ms());
                        }
                        if (fusaConfList[SM_INDEX_11].open_flag == true) {
                                // log_fusa("vga_consistence_check_start = %f\n",fusa_get_current_time_ms());
                                test_result = func_safety_sm_vga_consistence_check();
                                // log_fusa("vga_consistence_check_end = %f\n",fusa_get_current_time_ms());
                                if (test_result == true){
                                        func_safety_error_handler(fusaConfList[SM_INDEX_11].sm_num, fusaConfList[SM_INDEX_11].error_type);
                                }
                        }
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                }
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_2_CYCLE;
                break;
        case PERIODIC_SM_2_CYCLE:
                /* SM11 and SM13 */
                if ((fusaConfList[SM_INDEX_8].open_flag == true) || (fusaConfList[SM_INDEX_9].open_flag == true)) {
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                        if ((fusaConfList[SM_INDEX_8].open_flag == true) && (sample_adc_running_flag == false)) {
                                // log_fusa("rf_loopback_start = %f\n",fusa_get_current_time_ms());
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, false);
#endif
                                IRQ_value = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio,false);
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, true);
#endif
                                // log_fusa("rf_loopback_end = %f\n",fusa_get_current_time_ms());
                                if (IRQ_value){
                                        func_safety_error_handler(fusaConfList[SM_INDEX_8].sm_num, fusaConfList[SM_INDEX_8].error_type);
                                }
                        }
                        if (fusaConfList[SM_INDEX_9].open_flag == true)
                        {
                                // log_fusa("chirp_monitor_s =%f\n",fusa_get_current_time_ms());
                                fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
                                // log_fusa("chirp_monitor_e =%f\n",fusa_get_current_time_ms());
                        }

#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                }
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_3_CYCLE;
                break;
        case PERIODIC_SM_3_CYCLE:
                /* SM12 and SM14 */
                if ((fusaConfList[SM_INDEX_7].open_flag == true) || (fusaConfList[SM_INDEX_10].open_flag == true)) {
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                        if ((fusaConfList[SM_INDEX_7].open_flag == true) && (sample_adc_running_flag == false)) {
                                // log_fusa("if_loopback_start = %f\n",fusa_get_current_time_ms());
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, false);
#endif
                                IRQ_value = fmcw_radio_sm_if_loopback_IRQ(&bb->radio,false);
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, true);
#endif
                                // log_fusa("if_loopback_end = %f\n",fusa_get_current_time_ms());
                                if (IRQ_value){
                                        func_safety_error_handler(fusaConfList[SM_INDEX_7].sm_num, fusaConfList[SM_INDEX_7].error_type);
                                }
                        }

                        if (fusaConfList[SM_INDEX_10].open_flag == true)
                        {
                                // log_fusa("over_temp_detector_s =%f\n",fusa_get_current_time_ms());
                                fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio,false);
                                // log_fusa("over_temp_detector_e =%f\n",fusa_get_current_time_ms());
                        }
#if (INTER_FRAME_POWER_SAVE == 1)
                        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
                }
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_4_CYCLE;
                break;
        case PERIODIC_SM_4_CYCLE:
                /* SM101 Run Time LBIST for Baseband */
                if (fusaConfList[SM_INDEX_12].open_flag == true) {
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                        //disable sm8
                        func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, false);
#endif
                        // log_fusa("baseband_bist_s =%f\n",fusa_get_current_time_ms());
                        test_result = baseband_bist_ctrl(cfg->bb, false);
                        // log_fusa("baseband_bist_e =%f\n",fusa_get_current_time_ms());
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                        //enable sm8
                        func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, true);
#endif
                }

                if (get_track_cfg() != CAN_INT) {
                        /* SM120 CAN loopback test */
                        if (fusaConfList[SM_INDEX_21].open_flag == true) {
                                // log_fusa("can_loopback_start = %f\n",fusa_get_current_time_ms());
                                func_safety_sm_can_loopback_init();
                                // log_fusa("can_loopback_end = %f\n",fusa_get_current_time_ms());
                        }

                        /* SM126 SPI loopback test */
                        if (fusaConfList[SM_INDEX_27].open_flag == true) {
                                // log_fusa("spi_loopback_start = %f\n",fusa_get_current_time_ms());
                                func_safety_sm_spi_loopback(fusaConfList[SM_INDEX_27].error_type);
                                // log_fusa("spi_loopback_end = %f\n",fusa_get_current_time_ms());
                        }
                }

                /* SM129 Configuration Registers Protection */
                if (fusaConfList[SM_INDEX_28].open_flag == true) {
                        // log_fusa("reg_protection_start = %f\n",fusa_get_current_time_ms());
                        func_safety_sm_can_config_reg_protection(fusaConfList[SM_INDEX_28].error_type);
                        // log_fusa("reg_protection_end = %f\n",fusa_get_current_time_ms());
                }

                /* SM805 VGA Gain Consistence Check */
                if (fusaConfList[SM_INDEX_33].open_flag == true) {
                        // log_fusa("periodic_readback_configuration_start = %f\n",fusa_get_current_time_ms());
                        func_safety_sm_periodic_readback_configuration_registers(fusaConfList[SM_INDEX_33].error_type);
                        // log_fusa("periodic_readback_configuration_end = %f\n",fusa_get_current_time_ms());
                }

                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_1_CYCLE;
                periodic_sm_finish_flag = true;
                break;
        default:
                FUNC_SAFETY_PERIODIC_SM_CLCLE = PERIODIC_SM_1_CYCLE;
                break;
        }
}

/**
 * @name: fusa_run_periodic_item
 * @msg: run function safety item assigned by func_safety_sm_num
 * @param {int32_t} func_safety_sm_num
 * @param {uint32_t} param
 * @return {*}
 */
void fusa_run_periodic_item(int32_t func_safety_sm_num, uint32_t param)
{
#if SAFETY_PERIODIC_RUN_ITEMS_EN
    float time_ms = 0.0;
    baseband_t* bb = baseband_get_cur_bb();
    // uint8_t irq_val = 0;

#if ((SAFETY_FEATURE_SM101 == FEATURE_ON) && (SAFETY_FEATURE_SM104 == FEATURE_ON))
        clear_bb_memory_ecc_status_bit();
#endif
    switch (func_safety_sm_num)
    {
    //==================================================
#if (SAFETY_FEATURE_SM1 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM1:
        if(param == FUNC_SAFETY_ITEM_SM1_SET_PART)
        {
                func_safety_sm_ldo_monitor_set_part(&bb->radio);
        }
        else if(param == FUNC_SAFETY_ITEM_SM1_READ_PART)
        {
                func_safety_sm_ldo_monitor_read_part();
        }
        else
        {
                log_fusa("unsupport param: item [%d] param [0x%x]\n",func_safety_sm_num,param);
        }
        break;
#endif
    //==================================================
#if (SAFETY_FEATURE_SM6 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM6:
        time_ms = fusa_get_current_time_ms();
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_5_SHIFT);
        fmcw_radio_sm_rfpower_detector_IRQ(&bb->radio);
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        time_ms = fusa_get_current_time_ms() - time_ms;
        // log_fusa("sm6 execute time_ms = %f\n", time_ms );
        break;
#endif
    //==================================================
#if (SAFETY_FEATURE_SM11 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM11:
        time_ms = fusa_get_current_time_ms();
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        if (sample_adc_running_flag == false)
        {
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, false);
#endif
                if(fmcw_radio_sm_if_loopback_IRQ(&bb->radio,false))
                {
                        func_safety_error_handler(fusaConfList[SM_INDEX_7].sm_num, fusaConfList[SM_INDEX_7].error_type);
                }
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, true);
#endif
        }
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        time_ms = fusa_get_current_time_ms() - time_ms;
        // log_fusa("sm11 execute time_ms = %f\n", time_ms );
        break;
#endif
    //==================================================
#if (SAFETY_FEATURE_SM12 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM12:
        time_ms = fusa_get_current_time_ms();
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        if (sample_adc_running_flag == false)
        {
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, false);
#endif
                if(fmcw_radio_sm_rf_loopback_IRQ(&bb->radio,false))
                {
                        func_safety_error_handler(fusaConfList[SM_INDEX_8].sm_num, fusaConfList[SM_INDEX_8].error_type);
                }
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
                func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, true);
#endif
        }
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        time_ms = fusa_get_current_time_ms() - time_ms;
        // log_fusa("sm12 execute time_ms = %f\n", time_ms );
        break;
#endif
    //==================================================
#if (SAFETY_FEATURE_SM13 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM13:
        time_ms = fusa_get_current_time_ms();
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        fmcw_radio_sm_chirp_monitor_IRQ(&bb->radio);
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        time_ms = fusa_get_current_time_ms() - time_ms;
        // log_fusa("sm13 execute time_ms = %f\n", time_ms );
        break;
#endif
    //==================================================
#if (SAFETY_FEATURE_SM14 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM14:
        time_ms = fusa_get_current_time_ms();
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio,false);
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        time_ms = fusa_get_current_time_ms() - time_ms;
        // log_fusa("sm14 execute time_ms = %f\n", time_ms );
        break;
#endif
    //==================================================
#if (SAFETY_FEATURE_SM101 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM101:
        time_ms = fusa_get_current_time_ms();
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
        //disable sm8
        func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, false);
#endif
        baseband_bist_ctrl(&bb->bb_hw, false);
#if (SAFETY_FEATURE_SM8 == FEATURE_ON)
        func_safety_sm8_enable(fusaConfList[SM_INDEX_6].error_type, true);
#endif
        time_ms = fusa_get_current_time_ms() - time_ms;
        // log_fusa("sm101 execute time_ms = %f\n", time_ms );
        break;
#endif
    //==================================================
#if (SAFETY_FEATURE_SM201 == FEATURE_ON)
    case FUNC_SAFETY_ITEM_SM201:
        time_ms = fusa_get_current_time_ms();
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        if(func_safety_sm_vga_consistence_check())
        {
                func_safety_error_handler(fusaConfList[SM_INDEX_11].sm_num, fusaConfList[SM_INDEX_11].error_type);
        }
#if INTER_FRAME_POWER_SAVE
        baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        time_ms = fusa_get_current_time_ms() - time_ms;
        // log_fusa("sm201 execute time_ms = %f\n", time_ms );
        break;
#endif
    //==================================================
    default:
        log_fusa("unsupport or unopened item [%d] param [0x%x]\n",func_safety_sm_num,param);
        break;
    }
#else
        log_fusa("unsupport or unopened item [%d] param [0x%x]\n",func_safety_sm_num,param);
#endif
}

/**
 * @name: fusa_run_periodic_items_default
 * @msg: a demo for periodic run item
 * @param {*}
 * @return {*}
 */
void fusa_run_periodic_items_default(void)
{
    static int cycle_num = 0;
    switch (cycle_num)
    {
    case 0:
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM6,0);
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM201,0);
        break;
    case 1:
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM12,0);
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM13,0);
        break;
    case 2:
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM11,0);
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM14,0);
        break;
    case 3:
        fusa_run_periodic_item(FUNC_SAFETY_ITEM_SM101,0);
        break;
    default:
        log_fusa("\n");
        break;
    }
    cycle_num++;
    if(cycle_num > 3)
    {
        cycle_num = 0;
    }
}

#if (FUNC_SAFETY_CLI == FEATURE_ON)

/**
 * @name: bb_mem_ecc_err_injection
 * @msg:
 * @param {int} val : 1 single bit err inject, 2 double bit err inject
 * @return {*}
 */
void bb_mem_ecc_err_injection(int val)
{
    volatile uint32_t *p = (uint32_t *)0xE00000;
    uint32_t temp = 0;
    int32_t MEMORY_NUM = 10;
    if(val == 1)
    {
        for(int i = 0; i < MEMORY_NUM; i++)
        {
            log_fusa("BB MEM ERR inject SB, mux =  %d\r\n",i);
            raw_write_op(BB_REG_SYS_BNK_ACT,0x01);
            raw_write_op(BB_REG_SYS_MEM_ACT,i);
            raw_write_op(BB_REG_SYS_ECC_ENA,0x7fff);
            raw_read_op(BB_REG_SYS_IRQ_ENA);//random readback an bb reg,to avoid latency case BB_REG_SYS_ECC_ENA write isn't effective
            *p = 0x12345678;
            log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));
            raw_write_op(BB_REG_SYS_ECC_ENA,0);
            raw_read_op(BB_REG_SYS_IRQ_ENA);//random readback an bb reg,to avoid latency case BB_REG_SYS_ECC_ENA write isn't effective
            *p = 0x12345670;
            log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));
            raw_write_op(BB_REG_SYS_ECC_ENA,0x7fff);
            raw_read_op(BB_REG_SYS_IRQ_ENA);//random readback an bb reg,to avoid latency case BB_REG_SYS_ECC_ENA write isn't effective
            temp = *p;
            log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));
            fusa_delay_ms(1000);
        }
    }
    else if (val == 2)
    {
        for(int i = 0; i < MEMORY_NUM; i++)
        {
            log_fusa("BB MEM ERR inject SB, mux =  %d\r\n",i);
            raw_write_op(BB_REG_SYS_BNK_ACT,0x01);
            raw_write_op(BB_REG_SYS_MEM_ACT,i);
            raw_write_op(BB_REG_SYS_ECC_ENA,0x7fff);
            raw_read_op(BB_REG_SYS_IRQ_ENA);//random readback an bb reg,to avoid latency case BB_REG_SYS_ECC_ENA write isn't effective
            *p = 0x12345678;
            log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));
            raw_write_op(BB_REG_SYS_ECC_ENA,0);
            raw_read_op(BB_REG_SYS_IRQ_ENA);//random readback an bb reg,to avoid latency case BB_REG_SYS_ECC_ENA write isn't effective
            *p = 0x12345671;
            log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));
            raw_write_op(BB_REG_SYS_ECC_ENA,0x7fff);
            raw_read_op(BB_REG_SYS_IRQ_ENA);//random readback an bb reg,to avoid latency case BB_REG_SYS_ECC_ENA write isn't effective
            temp = *p;
            log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));
            fusa_delay_ms(1000);
        }
    }
    else
    {
        log_fusa("[ERR]BB MEM ERR inject val must be 1(SB) or 2(DB),input val=%d\r\n",val);
    }
//     log_fusa("temp = 0x%08x\r\n",temp);
}
void fusa_reg_dump(uint32_t base_addr, uint32_t reg_nums)
{
        for(uint32_t i = 0; i < reg_nums; i++)
        {
                log_fusa("reg_addr = 0x%08x  val = 0x%08x\n",base_addr,raw_read_op(base_addr));
                base_addr += 4;
        }
}
void fusa_safety_module_reg_dump(void)
{
        log_fusa("==================================================================\n");
        fusa_reg_dump(REL_REGBASE_EMU,8);
        fusa_reg_dump(REL_REGBASE_EMU + 0X100,7);
        fusa_reg_dump(REL_REGBASE_EMU + 0X200,6);
        fusa_reg_dump(REL_REGBASE_EMU + 0X300,13);
        fusa_reg_dump(REL_REGBASE_EMU + 0X400,13);
        fusa_reg_dump(REL_REGBASE_EMU + 0X500,15);
        fusa_reg_dump(REL_REGBASE_EMU + 0X600,16);
}
/************************************************************************************
;
; This routine:
;       set chirp number
; arg:
;       radio
;       size0: [int] value write into FMCW_CHIRP_SIZE_1_0
;       size1: [int] value write into FMCW_CHIRP_SIZE_1_1
; return:
;       void
; Change tracking:
;       Ver0.0.1 :
;***********************************************************************************/
void fmcw_radio_set_chirp_size(fmcw_radio_t *radio, int size0, int size1)
{
    uint8_t old_bank = fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);
    fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_0, size0);
    fmcw_radio_reg_write(radio, R5_FMCW_CHIRP_SIZE_1_1, size1);
    fmcw_radio_switch_bank(radio, old_bank);
}
/**
 * @name: fmcw_lockstep_err_injection
 * @msg:
 * @param {fmcw_radio_t} *radio
 * @return {*}
 */
uint8_t fmcw_lockstep_err_injection(fmcw_radio_t *radio)
{
    uint8_t ret      = 0;
    uint8_t old_bank = fmcw_radio_switch_bank(NULL,10);
//     safety_en(FIELD_ANA,TYPE_IRQ,SAFETY_MASK(ANA_FMCW_LS),DISABLE);
//     func_safety_clear_reg_emu_dig_err_irq_ena(FUNC_SAFETY_BIT_14_SHIFT);
    fmcw_radio_reg_write(NULL,0x70,0x00);//disable lockstep
    fmcw_radio_set_chirp_size(radio, 20 , 0);
    fmcw_radio_start_fmcw(radio);
    fmcw_radio_reg_write(NULL,0x70,0x08);//enable lockstep
    fmcw_radio_reg_write(NULL,0x70,0x0c);//release IRQ reset
    fmcw_radio_reg_write(NULL,0x70,0x0e);//start lockstep
    fmcw_radio_reg_write(NULL,0x70,0x0c);//start lockstep
    fusa_delay_ms(1);
    if(fmcw_radio_reg_read(radio,0x71)&0x01)
    {
        ret = 1;
    }
    fmcw_radio_reg_write(NULL,0x70,0x0d);//clear irq
    fmcw_radio_reg_write(NULL,0x70,0x0c);
    fmcw_radio_switch_bank(NULL,old_bank);
    fmcw_radio_set_chirp_size(radio, REG_L(radio->nchirp), REG_M(radio->nchirp));
    return ret;
}
void can_ecc_err_injection(int32_t can_id)
{
        uint32_t addr_base = (can_id == CAN_0_ID)? REL_REGBASE_CAN0 : REL_REGBASE_CAN1;
         /* disable ecc*/
        //set CAN_0.MCR.CFG to 0x1
        uint32_t temp = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
        raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, temp | BIT_MODE_CTRL_CFG);
        //set CAN_0.MCR.ECCENA to 0x0
        temp = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
        raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, temp & (~BIT_MODE_CTRL_ECCENA));
        //set CAN_0.MCR.CFG to 0x0
        temp = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
        raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, temp & (~BIT_MODE_CTRL_CFG));

        raw_write_op(addr_base + REG_CAN_TX_BUFFER_OFFSET,0x12345678);

        /* enable ecc*/
        //set CAN_0.MCR.CFG to 0x1
        temp = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
        raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, temp | BIT_MODE_CTRL_CFG);
        //set CAN_0.MCR.ECCENA to 0x1
        temp = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
        raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, temp | BIT_MODE_CTRL_ECCENA);
        //set CAN_0.MCR.CFG to 0x0
        temp = raw_read_op(addr_base + REG_CAN_MODE_CTRL_OFFSET);
        raw_write_op(addr_base + REG_CAN_MODE_CTRL_OFFSET, temp & (~BIT_MODE_CTRL_CFG));

        raw_read_op(addr_base + REG_CAN_TX_BUFFER_OFFSET);
}
void func_safety_err_inject_handler(int32_t err_item, int32_t param)
{
    uint32_t temp = 0;
    uint32_t reg_val = 0;
    volatile uint32_t *p = NULL;
    baseband_t* bb = baseband_get_cur_bb();
    log_fusa("[err_inject]item = %d; time = %f\n",err_item,fusa_get_current_time_ms());
    switch (err_item)
    {
    case FUNC_SAFETY_ITEM_SM1  :
        /* store old reg value*/
        fmcw_radio_switch_bank(NULL, 10);
        temp = RADIO_READ_BANK_REG(10, LDO_MON_VTHSEL);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTHSEL, 0x00);
        fmcw_radio_switch_bank(NULL, 1);
        reg_val = fmcw_radio_reg_read_field(NULL,R1_ADC_LDO0,R1_ADC_LDO0_LDO11_ADC12_VO_SEL_S,R1_ADC_LDO0_LDO11_ADC12_VO_SEL_M);
        /* err inject*/
        fmcw_radio_reg_mod(NULL,R1_ADC_LDO0,R1_ADC_LDO0_LDO11_ADC12_VO_SEL_S,R1_ADC_LDO0_LDO11_ADC12_VO_SEL_M,0X0);
        fusa_delay_ms(300);
        /* recover */
        fmcw_radio_reg_mod(NULL,R1_ADC_LDO0,R1_ADC_LDO0_LDO11_ADC12_VO_SEL_S,R1_ADC_LDO0_LDO11_ADC12_VO_SEL_M,reg_val);
        fmcw_radio_switch_bank(NULL, 10);
        RADIO_WRITE_BANK_REG(10, LDO_MON_VTHSEL, temp);
        break;
    case FUNC_SAFETY_ITEM_SM2  :
        // /* read monitor status 1 & 0*/
        // fmcw_radio_switch_bank(NULL, 10);
        // temp = RADIO_READ_BANK_REG(10,ITF_IRQ_1);
        // fmcw_radio_switch_bank(NULL, 0);
        // reg_val = RADIO_READ_BANK_REG(0,MS);
        // log_fusa("Monitot status_1 = 0x%02x  status_0 = 0x%02x\n",temp,reg_val);
        // /* store old reg value*/
        // fmcw_radio_switch_bank(NULL, 0);
        // temp = RADIO_READ_BANK_REG(0, CBC33_MONITOR0);
        // reg_val = RADIO_READ_BANK_REG(0, CBC33_MONITOR1);
        // /* err inject*/
        // RADIO_WRITE_BANK_REG(0, CBC33_MONITOR0, 0x00);
        // RADIO_WRITE_BANK_REG(0, CBC33_MONITOR1, 0x00);

        // fusa_delay_ms(2);
        // /* read monitor status 1 & 0*/
        // fmcw_radio_switch_bank(NULL, 10);
        // temp = RADIO_READ_BANK_REG(10,ITF_IRQ_1);
        // fmcw_radio_switch_bank(NULL, 0);
        // reg_val = RADIO_READ_BANK_REG(0,MS);
        // log_fusa("Monitot status_1 = 0x%02x  status_0 = 0x%02x\n",temp,reg_val);
        // fusa_delay_ms(2);
        // /* recover */
        // fmcw_radio_switch_bank(NULL, 0);
        // RADIO_WRITE_BANK_REG(0, CBC33_MONITOR0, temp);
        // RADIO_WRITE_BANK_REG(0, CBC33_MONITOR1, reg_val);
        fmcw_radio_sm_avdd33_monitor_IRQ(NULL, true);
        break;
    case FUNC_SAFETY_ITEM_SM3  :
        // /* store old reg value*/
        // fmcw_radio_switch_bank(NULL, 0);
        // temp = RADIO_READ_BANK_REG(0, DVDD11_MONITOR);
        // /* err inject*/
        // RADIO_WRITE_BANK_REG(0, DVDD11_MONITOR, temp & 0x07);
        // fusa_delay_ms(2);
        // /* recover */
        // fmcw_radio_switch_bank(NULL, 0);
        // RADIO_WRITE_BANK_REG(0, DVDD11_MONITOR, temp);
        fmcw_radio_sm_dvdd11_monitor_IRQ(NULL, true);
        break;
    case FUNC_SAFETY_ITEM_SM4  :
        fmcw_radio_sm_bg_monitor_IRQ(NULL,true);
        break;
    case FUNC_SAFETY_ITEM_SM5  :
        fusa_delay_ms(100);
        fmcw_radio_switch_bank(NULL, 0);
        temp = RADIO_READ_BANK_REG(0, RP_DIV);
        RADIO_WRITE_BANK_REG(0, RP_DIV, temp & 0xf8);
        fusa_delay_ms(20);
        fmcw_radio_switch_bank(NULL, 0);
        RADIO_WRITE_BANK_REG(0, RP_DIV, temp);
        fusa_delay_ms(20);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        //configure radio
        fmcw_radio_reg_write(NULL, 0, 0);
        fmcw_radio_reg_write(NULL, 0x7d, 0x05);
        fmcw_radio_reg_write(NULL, 0x7d, 0x15);
        fmcw_radio_reg_write(NULL, 0, 0xa);
        fmcw_radio_reg_write(NULL, 0x6e, 0x0);
        fmcw_radio_reg_write(NULL, 0x6e, 0x2);
        break;
    case FUNC_SAFETY_ITEM_SM6  :
        fmcw_radio_sm_rfpower_detector_fault_injection_threshold(NULL);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM8  :
        func_safety_clear_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
        temp = fmcw_radio_sm_saturation_detector_fault_injection_IRQ(NULL);
        func_safety_set_reg_emu_rf_err_irq_ena(FUNC_SAFETY_BIT_7_SHIFT);
        log_fusa("[fusa err inject]item = %d, irq_val = %d\n",err_item,temp);
        break;
    case FUNC_SAFETY_ITEM_SM11 :
        temp = fmcw_radio_sm_if_loopback_IRQ(&bb->radio,true);
        if (temp)
        {
                func_safety_error_handler(fusaConfList[SM_INDEX_7].sm_num, fusaConfList[SM_INDEX_7].error_type);
        }
        log_fusa("[fusa err inject]item = %d,irq_val = %d\n",err_item,temp);
        break;
    case FUNC_SAFETY_ITEM_SM12 :
        temp = fmcw_radio_sm_rf_loopback_IRQ(&bb->radio,true);
        if (temp)
        {
                func_safety_error_handler(fusaConfList[SM_INDEX_8].sm_num, fusaConfList[SM_INDEX_8].error_type);
        }
        log_fusa("[fusa err inject]item = %d,irq_val = %d\n",err_item,temp);
        break;
    case FUNC_SAFETY_ITEM_SM13 :
        fmcw_radio_sm_chirp_monitor_fault_injection_IRQ(&bb->radio);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM14 :
        fmcw_radio_sm_over_temp_detector_IRQ(&bb->radio,true);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM101:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM102:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        core_reg_bit_set(REG_CPU_ERP_CTRL,0);
        p = (uint32_t *)0xa07ff0;
        *p = 0x12345678;
        core_reg_bit_clr(REG_CPU_ERP_CTRL,0);
        *p = 0x12345679;
        core_reg_bit_set(REG_CPU_ERP_CTRL,0);
        temp = *p;
        log_fusa("[fusa err inject]item = %d ,temp = 0x%x\n",err_item,temp);
        break;
    case FUNC_SAFETY_ITEM_SM103:
        core_reg_bit_set(REG_CPU_ERP_CTRL,2);
        p = (uint32_t *)0x77fff0;
        *p = 0x12345678;
        core_reg_bit_clr(REG_CPU_ERP_CTRL,2);
        *p = 0x12345679;
        core_reg_bit_set(REG_CPU_ERP_CTRL,2);
        temp = *p;
        log_fusa("[fusa err inject]item = %d ,temp = 0x%x\n",err_item,temp);
        break;
    case FUNC_SAFETY_ITEM_SM104:
        bb_mem_ecc_err_injection(1);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM105:
        bb_mem_ecc_err_injection(2);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM106:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM107:
        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET,1);
        raw_read_op(REL_REGBASE_DMU + REG_DMU_IRQ_STA_OFFSET);  //must have a read or write or a delay operation,if not, can't inject err success
        // fusa_safety_module_reg_dump();
        p = (uint32_t *)0x77fff0;
        raw_write_op((uint32_t)(p),0x12345678);
        log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));

        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET,0);
        raw_read_op(REL_REGBASE_DMU + REG_DMU_IRQ_STA_OFFSET);
        // fusa_safety_module_reg_dump();
        raw_write_op((uint32_t)(p),0x12345671);
        log_fusa("temp = 0x%x\n",raw_read_op((uint32_t)(p)));

        raw_write_op(REL_REGBASE_DMU + REG_DMU_SYS_MEMRUN_ENA_OFFSET,1);
        raw_read_op(REL_REGBASE_DMU + REG_DMU_IRQ_STA_OFFSET);
        // fusa_safety_module_reg_dump();
        temp = raw_read_op((uint32_t)(p));
        log_fusa("[fusa err inject]item = %d ,temp = 0x%x\n",err_item,temp);
        break;
    case FUNC_SAFETY_ITEM_SM108:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM109:
        can_ecc_err_injection(param);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM110:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM112:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM113:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM114:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM117:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM118:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM120:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM121:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM122:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM123:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM124:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM125:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM126:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM127:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM128:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM129:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM130:
        fmcw_lockstep_err_injection(&bb->radio);
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM133:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM201:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM202:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM203:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM204:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM205:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM206:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM207:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    case FUNC_SAFETY_ITEM_SM805:
        log_fusa("[fusa err inject]item = %d\n",err_item);
        break;
    default:
        log_fusa("unkonw item = %d\n",err_item);
        break;
    }
}

#endif
