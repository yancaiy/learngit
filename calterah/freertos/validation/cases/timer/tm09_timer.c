#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "dw_timer.h"
#include "timer_hal.h"
#include "arc_timer.h"
#include "arc.h"
#include "arc_builtin.h"
#include "arc_wdg.h"
#include "tm09_timer.h"
//#include "dw_wdt.h"
#include "validation.h"
#include "vmodule.h"

#define timer_log EMBARC_PRINTF
#define TIMER_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_TIMER_ID, cid, f_init, NULL)

#define CORE_FREQUENCY      300000000

int times_interrupt = 0;
int times_interrupt_expire = 0;
int timer_mode = USER_DEFINED_MODE;
int timer_priod = 0;
int timerTestState = 0;

static void timer0_func(void *param)
{

    uint64_t time;
    float ftime;

    time = timer_rtc_count();
    ftime = time / (CORE_FREQUENCY/1000);
    timer_log("time0_interupt = %d,time = %f ms\r\n",times_interrupt++, ftime);

    if(times_interrupt >= times_interrupt_expire)
    {
        dw_timer_stop(0);
        times_interrupt = 0;
        times_interrupt_expire = 0;
        timerTestState = 0;
    }
    else
    {
        if(timer_mode == FREE_RUNNING_MODE)
        {
            dw_timer_t *dw_timer = (dw_timer_t *)param;
            dw_timer_ops_t *timer_ops = (dw_timer_ops_t *)dw_timer->ops;
            timer_ops->enable(dw_timer,0,0);
            timer_ops->load_count_update(dw_timer,0,timer_priod);
            timer_ops->enable(dw_timer,0,1);
        }
    }
}

static void timer1_func(void *param)
{

    uint64_t time;
    float ftime;

    time = timer_rtc_count();
    ftime = time / (CORE_FREQUENCY/1000);
    timer_log("time1_interupt = %d,time = %f ms\r\n",times_interrupt++, ftime);

    if(times_interrupt >= times_interrupt_expire)
    {
        dw_timer_stop(1);
        times_interrupt = 0;
        times_interrupt_expire = 0;
        timerTestState = 0;
    }
    else
    {
        if(timer_mode == FREE_RUNNING_MODE)
        {
            dw_timer_t *dw_timer = (dw_timer_t *)param;
            dw_timer_ops_t *timer_ops = (dw_timer_ops_t *)dw_timer->ops;
            timer_ops->enable(dw_timer,1,0);
            timer_ops->load_count_update(dw_timer,1,timer_priod);
            timer_ops->enable(dw_timer,1,1);
        }
    }
}

static void timer2_func(void *param)
{

    uint64_t time;
    float ftime;

    time = timer_rtc_count();
    ftime = time / (CORE_FREQUENCY/1000);
    timer_log("time2_interupt = %d,time = %f ms\r\n",times_interrupt++, ftime);

    if(times_interrupt >= times_interrupt_expire)
    {
        dw_timer_stop(2);
        times_interrupt = 0;
        times_interrupt_expire = 0;
        timerTestState = 0;
    }
    else
    {
        if(timer_mode == FREE_RUNNING_MODE)
        {
            dw_timer_t *dw_timer = (dw_timer_t *)param;
            dw_timer_ops_t *timer_ops = (dw_timer_ops_t *)dw_timer->ops;
            timer_ops->enable(dw_timer,2,0);
            timer_ops->load_count_update(dw_timer,2,timer_priod);
            timer_ops->enable(dw_timer,2,1);
        }
    }
}

static void timer3_func(void *param)
{

    uint64_t time;
    float ftime;

    time = timer_rtc_count();
    ftime = time / (CORE_FREQUENCY/1000);
    timer_log("time3_interupt = %d,time = %f ms\r\n",times_interrupt++, ftime);

    if(times_interrupt >= times_interrupt_expire)
    {
        dw_timer_stop(3);
        times_interrupt = 0;
        times_interrupt_expire = 0;
        timerTestState = 0;
    }
    else
    {
        if(timer_mode == FREE_RUNNING_MODE)
        {
            dw_timer_t *dw_timer = (dw_timer_t *)param;
            dw_timer_ops_t *timer_ops = (dw_timer_ops_t *)dw_timer->ops;
            timer_ops->enable(dw_timer,3,0);
            timer_ops->load_count_update(dw_timer,3,timer_priod);
            timer_ops->enable(dw_timer,3,1);
        }
    }
}
typedef void (*timer_func_callback)(void *);

static timer_func_callback timer_func[] =
{
    timer0_func,
    timer1_func,
    timer2_func,
    timer3_func,
};

extern dw_timer_t *dw_timer;
int32_t timer_test(uint32_t id,uint32_t mode,uint32_t period_ms,uint32_t times)
{

    int32_t result = E_OK;

    do {

        result = dw_timer_init();
        if (E_OK != result) {
            break;
        }
        result = dw_timer_mode_config(id,mode);
        if (E_OK != result) {
            break;
        }

        times_interrupt = 0;
        times_interrupt_expire = times;
        timer_mode = mode;
        timer_priod = period_ms * 100000;
        result = dw_timer_start(id , timer_priod , timer_func[id],dw_timer);
        if (E_OK != result) {
            break;
        }
        timerTestState = 1;

    } while (0);

    return result;
}

int32_t wait_timer_test_end(int timeout_ms)
{
    while(timerTestState == 1)
    {
        chip_hw_mdelay(1);
        if(timeout_ms-- <= 0)
        {
            return E_TMOUT;
        }
    }
    return E_OK;
}






/* TODO: add test cases */
void register_val_print()
{
    // system_enable(1);
    // raw_writel(0xa0000040, 0x1);
    // raw_writel(0xa0000054, 0x55AA);
    // timer_log("WDT_WDT_COMP_PARAM_1:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_COMP_PARAM_1_OFFSET));
    // timer_log("WDT_WDT_COMP_PARAM_2:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_COMP_PARAM_2_OFFSET));
    // timer_log("WDT_WDT_COMP_PARAM_3:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_COMP_PARAM_3_OFFSET));
    // timer_log("WDT_WDT_COMP_PARAM_4:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_COMP_PARAM_4_OFFSET));
    // timer_log("WDT_WDT_COMP_PARAM_5:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_COMP_PARAM_5_OFFSET));
    // timer_log("REG_DW_WDT_COMP_VERSION:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_COMP_VERSION_OFFSET));
    // timer_log("REG_DW_WDT_COMP_TYPE:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_COMP_TYPE_OFFSET));
    // timer_log("REG_DW_WDT_PROT_LEVEL:0x%08x\r\n",raw_readl(REG_DW_WDT_BASE+REG_DW_WDT_PROT_LEVEL_OFFSET));
}
int32_t timer_case_handle(void *self, void *params, uint32_t len)
{
    validation_case_t *vscase_ptr = self;

    int32_t result = E_OK;
    timer_auto_test_config_t config;
    param_analy(params,len,(auto_test_param_t *)&config);
    timer_log("vscase_ptr->case_id=%d!\r\n",vscase_ptr->case_id);
    chip_hw_mdelay(100);
    // register_val_print();
    switch (vscase_ptr->case_id)
    {
    case 1:
        result = timer_test(config.id,USER_DEFINED_MODE,config.period_ms,config.times);
        result = wait_timer_test_end(config.times * config.period_ms + 5000);
        break;
    case 2:
        result = timer_test(config.id,FREE_RUNNING_MODE,config.period_ms,config.times);
        timer_log("vscase_ptr->case_id=%d!\r\n",vscase_ptr->case_id);
        result = wait_timer_test_end(config.times * config.period_ms + 5000);
        break;
    default:
        return -11;
        break;
    }
    return result;
}

TIMER_VALIDATION_SYS_CASE(TIMER_CID_1, timer_case_handle);
TIMER_VALIDATION_SYS_CASE(TIMER_CID_2, timer_case_handle);

