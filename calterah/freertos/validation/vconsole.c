#include "embARC.h"
#include <string.h>
#include <stdarg.h>
#include "embARC_toolchain.h"
#include "embARC_debug.h"

#include "uart_hal.h"


int32_t vconsole_get(uint8_t *data, uint32_t len)
{
	int32_t result = E_OK;

	if ((NULL == data) || (0 == len)) {
		result = E_PAR;
	} else {
		uint32_t id = CHIP_CONSOLE_UART_ID;
		result = uart_read(id, data, len);
	}

	return result;
}


/* register pre/post printf to xprintf module. */
static int32_t vconsole_pre_printf(void)
{
	return arc_lock_save();
}

static void vconsole_post_printf(int32_t sts, uint32_t reserved1)
{
	arc_unlock_restore((uint32_t)sts);
}

void vconsole_init(void)
{
	xprintf_callback_install(vconsole_pre_printf, vconsole_post_printf);
}
