#include "arc.h"
#include "arc_builtin.h"
#include "arc_wdg.h"
#include "alps_emu_reg.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_error.h"
#include "alps_mmap.h"


bool arc_wdt_on_flag = false;

void arc_wdg_init(wdg_event_t event, uint32_t period)
{
    uint32_t value;

    //set EMU.DIG_ERR_SS1_MARK.CPU_WDT bit to 0x1
    value = raw_readl(REG_EMU_DIG_ERR_SS1_MASK);
    raw_writel(REG_EMU_DIG_ERR_SS1_MASK, value | (1 << 10));
    //set EMU.DIG_ERR_SS1_ENA.CPU_WDT bit to 0x1
    value = raw_readl(REG_EMU_DIG_ERR_SS1_ENA);
    raw_writel(REG_EMU_DIG_ERR_SS1_ENA, value | (1 << 10));

	_arc_aux_write(AUX_WDG_PASSWD, AUX_WDG_PERIOD_WEN_MASK);
	_arc_aux_write(AUX_WDG_PERIOD, period);

	_arc_aux_write(AUX_WDG_PASSWD, AUX_WDG_CTRL_WEN_MASK);
	_arc_aux_write(AUX_WDG_CTRL, ((event & 0x3) << 1) | AUX_WDG_ENABLE);

	_arc_aux_write(AUX_WDG_PASSWD, AUX_WDG_WR_DIS_MASK);
}

void arc_wdg_deinit(void)
{
	_arc_aux_write(AUX_WDG_PASSWD, AUX_WDG_CTRL_WEN_MASK);
	_arc_aux_write(AUX_WDG_CTRL, AUX_WDG_DISABLE);
}

void arc_wdg_update(uint32_t period)
{
    /* According to ARC data book, AUX_WDG_PASSWD register can only be wrote and can not be read */
//	if (_arc_aux_read(AUX_WDG_PASSWD) == AUX_WDG_PERIOD_AUTO_WEN_MASK) {
//		_arc_aux_write(AUX_WDG_PERIOD, period);
//	} else {
		_arc_aux_write(AUX_WDG_PASSWD, AUX_WDG_PERIOD_WEN_MASK);
		_arc_aux_write(AUX_WDG_PERIOD, period);
		_arc_aux_write(AUX_WDG_PASSWD, AUX_WDG_WR_DIS_MASK);
//	}
}

void arc_wdg_status_clear(void)
{
	uint32_t val = _arc_aux_read(AUX_WDG_CTRL);
	val &= ~(1 << 3);
	_arc_aux_write(AUX_WDG_CTRL, val);
}

uint32_t arc_wdg_count(void)
{
	return _arc_aux_read(AUX_WDG_COUNT);
}

/****************************************************
 * NAME         : arc_wdg_feed_wdt
 * DESCRIPTIONS : feed arc wdt
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
int32_t arc_wdg_feed_wdt(void)
{
        int32_t result = E_OK;

        /* setting the timeout is 200ms */
        uint32_t period = (200 * PERIOD_TO_MS);

        uint32_t time_out = 0;

        time_out = (_arc_aux_read(AUX_WDG_CTRL) >> 3);
        if (time_out) {
                uint8_t root_cnt = raw_readl(REG_EMU_REBOOT_CNT);

                EMBARC_PRINTF(
                                "[%s] [%d] timeout!!! root_cnt:%d, wdg_ctrl:0x%x wdg_count:0x%x!\r\n",
                                __func__, __LINE__, root_cnt, _arc_aux_read(AUX_WDG_CTRL),
                                _arc_aux_read(AUX_WDG_COUNT));

                raw_writel(EMU_SAVE_WDG_TO_MEM_ADDR, _arc_aux_read(AUX_WDG_CTRL));
                raw_writel(EMU_SAVE_WDG_TO_MEM_ADDR + 0x4,
                                _arc_aux_read(AUX_WDG_COUNT));
                raw_writel(EMU_SAVE_WDG_TO_MEM_ADDR + 0x8, root_cnt);

                arc_wdg_status_clear();
        }

        arc_wdg_update(period);

        return result;
}

/****************************************************
 * NAME         : arc_wdg_check_emu_error_code
 * DESCRIPTIONS : check arc wdg emu error_code, used when system initial
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void arc_wdg_check_emu_error_code(void)
{
        uint32_t value;

        //check emu reboot cnt
        EMBARC_PRINTF("\r\n");
        value = raw_readl(REG_EMU_REBOOT_CNT);
        EMBARC_PRINTF("EMU_REBOOT_CNT_READ: 0x%x, value: %d.\n", REG_EMU_REBOOT_CNT,
                        value);
        EMBARC_PRINTF("\r\n");
        //check emu digital SS1 error code
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE0);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE0: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE0, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE1);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE1: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE1, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE2);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE2: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE2, value);
        value = raw_readl(REG_EMU_DIG_ERR_SS1_CODE3);
        EMBARC_PRINTF("EMU_DIG_ERR_SS1_CODE3: 0x%x, value: 0x%x.\n",
                        REG_EMU_DIG_ERR_SS1_CODE3, value);
        EMBARC_PRINTF("\r\n");
}

/****************************************************
 * NAME         : arc_wdg_fault_injection
 * DESCRIPTIONS : arc wdg fault injection, stop feed wdt
 * INPUT        : None
 * OUTPUT       : None
 *****************************************************/
void arc_wdg_fault_injection(void)
{
        arc_wdt_on_flag = false;
}

