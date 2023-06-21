#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "routine.h"
#include "flash.h"

uint8_t routine_nor_flash_erase(uint8_t *params, uint32_t params_len)
{
	uint8_t rsp_code = 0;

	uint32_t idx, mem_addr = 0, mem_length = 0;

	if ((NULL == params) || (8 != params_len)) {
		rsp_code = 0xef;
	} else {
		for (idx = 0; idx < 4; idx++) {
			mem_addr |= (*params++ << ((3 - idx) * 8));
		}
		for (idx = 0; idx < 4; idx++) {
			mem_length |= (*params++ << ((3 - idx) * 8));
		}

		if(0 != flash_memory_erase(mem_addr, mem_length)) {
			rsp_code = 0xee;
		}else{
		       set_flash_addr(mem_addr);
		}
	}

	return rsp_code;
}
