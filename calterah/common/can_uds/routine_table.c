#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "routine.h"


/* nor flash erase routine. */
static routine_func_t nor_flash_erase[RC_TYPE_NUM_MAX - 1] = {
	{
		.func_id = RC_START_ROUTINE,
		.record_len = 8,
		.func = routine_nor_flash_erase
	},
	{
		.func_id = RC_STOP_ROUTINE,
		.record_len = 0,
		.func = NULL
	},
	{
		.func_id = RC_REQUEST_ROUTINE_RESULT,
		.record_len = 0,
		.func = NULL
	}
};


static uds_routine_t routine_list[ROUTINE_ID_MAX] = {
	{
		.id = RID_NOR_FLASH_ERASE,
		.func_table = nor_flash_erase
	}
};

uds_routine_t *uds_routines_get(uint32_t *cnt)
{
	if (cnt) {
		*cnt = sizeof(routine_list) / sizeof(uds_routine_t);
	}
	return routine_list;
}
