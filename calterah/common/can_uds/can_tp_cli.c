#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_error.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "board.h"

#ifdef OS_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#endif
#include "can_hal.h"
#include "can_if.h"
#include "can_tp.h"


#if 0
void cantp_cli_register(void)
{
}

void can_bus_init(void)
{
	can_config_t *config = (can_config_t *)can_config_get();
	if (NULL != config) {
		can_init(config);
		canif_init();
		cantp_init();
		canif_callback_register(0, cantp_confirmation, 0, cantp_indication);
		cantp_cli_register();
	}
}
#endif
