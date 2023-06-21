#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"
#include "arc_wdg.h"
#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif

static void watchdog_isr(void *params)
{
	/* TODO: customer in future! */
	while (1);
}

void watchdog_init(void)
{
	uint32_t period = (200 * PERIOD_TO_MS);

	int_disable(INT_WDG_TIMEOUT);

	int_handler_install(INT_WDG_TIMEOUT, watchdog_isr);

	arc_wdg_init(WDG_EVENT_TIMEOUT, period);

	int_enable(INT_WDG_TIMEOUT);

	arc_wdt_on_flag = true;
}

