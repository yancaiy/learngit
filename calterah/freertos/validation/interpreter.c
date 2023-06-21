#include <string.h>
#include "embARC.h"
#include "embARC_debug.h"
#include "validation.h"
#include "interpreter.h"
#include "vconsole.h"


/*************************************************
 * start *
 * stop  *
 *  test  :  mid  :  cid  :  length  :  payload  *
 *************************************************/
int32_t interpreter_case_parameter(uint16_t *mid, uint16_t *cid, uint8_t **data, uint32_t *len)
{
	int32_t result = E_OK;

	uint16_t mod_id, vcase_id;
	uint32_t payload_len = 0, dbuf_len = 0;
	uint8_t *payload_ptr = NULL;

	do {
		if ((NULL == mid) || (NULL == cid) || \
		    ((NULL == data) || (NULL == *data)) || \
		    ((NULL == len) || (5 > *len))) {
			result = E_PAR;
			break;
		}

		payload_ptr = *data;
		dbuf_len = *len;
		vconsole_get(payload_ptr, 4);
		payload_ptr[5] = '\0';

		if (0 == strcmp((const char *)payload_ptr, "stop")) {
			result = VSCASE_STOP;
			EMBARC_PRINTF("VSCASE_STOP\r\n");
			break;
		}
		if (0 == strcmp((const char *)payload_ptr, "star")) {
			vconsole_get(payload_ptr, 1);
			if (payload_ptr[0] == 't') {
				result = VSCASE_START;
				EMBARC_PRINTF("VSCASE_START\r\n");
			} else {
				result = E_SYS;
			}
			break;
		}
		if (strcmp((const char *)payload_ptr, "test")) {
			result = E_SYS;
			break;
		}

		vconsole_get(payload_ptr, 8);
		mod_id = payload_ptr[0] << 8;
		mod_id |= payload_ptr[1];
		vcase_id = payload_ptr[2] << 8;
		vcase_id |= payload_ptr[3];
		payload_len = payload_ptr[4] << 24;
		payload_len |= payload_ptr[5] << 16;
		payload_len |= payload_ptr[6] << 8;
		payload_len |= payload_ptr[7];

		*mid = mod_id;
		*cid = vcase_id;

		if (payload_len > dbuf_len) {
			result = E_SYS;
			break;
		}

		/* clean buffer */
		for (uint8_t i = 0; i < 8; i++) {
			payload_ptr[i] = 0x0;
		}

		vconsole_get(payload_ptr, payload_len);
		*len = payload_len;
	} while (0);

	return result;
}


/* only call in task context. */
int32_t interpreter_vscase_stop(void)
{
	int32_t result = E_OK;

	uint8_t vcase_msg[8];

	memset(vcase_msg, 0, 8);
	vconsole_get(vcase_msg, 4);
	vcase_msg[4] = '\0';
	if (0 == strcmp((const char *)vcase_msg, "stop")) {
		result = 1;
	}

	return result;
}
