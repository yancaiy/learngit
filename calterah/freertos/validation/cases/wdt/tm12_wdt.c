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
#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "dw_wdt.h"
#include "tm12_wdt.h"

uint32_t apb_period = 0xa;//6s->wdt_clk = 10M
/*apb_wdt_reset*/
void apb_wdt_test_case1(void) 
{

	system_enable(1);
	raw_writel(0xa0000040, 0x1);
	raw_writel(0xa0000050, 0x55AA);

	dw_wdt_init(APB_WDT_EVENT_RESET, 1, apb_period);

}
/*apb_wdt_INT*/
void apb_wdt_test_case2(void) 
{

	wdt_int_enable();

	dw_wdt_init(APB_WDT_EVENT_INT, 1, apb_period);

}