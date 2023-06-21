#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "routine.h"

#define RC_NRC_SUB_FUNC_NOT_SUPPORT	(0x12)
#define RC_NRC_INCORRECT_MSG_LEN	(0x13)
#define RC_NRC_CONDITIONS_NOT_CORRECT	(0x22)
#define RC_NRC_REQUEST_SEQ_ERROR	(0x24)
#define RC_NRC_REQUEST_OUT_OF_RANGE	(0x31)
#define RC_NRC_SECURITY_ACCESS_DENIED	(0x33)
#define RC_NRC_GENERAL_PROG_FAIL	(0x72)


static uint8_t uds_start_routine(uint16_t rid, pdu_t *rxdata, pdu_t *txdata)
{
	uint8_t rsp_code = 0;
	uint16_t record_length = 0;
	uint32_t idx, routine_cnt = 0;

	routine_callback start_func = NULL;
	routine_func_t *func_list = NULL;
	uds_routine_t *routine_list = uds_routines_get(&routine_cnt);

	if (NULL == routine_list) {
		rsp_code = RC_NRC_REQUEST_OUT_OF_RANGE;
	} else {
		for (idx = 0; idx < routine_cnt; idx++) {
			if (rid == routine_list[idx].id) {
				func_list = routine_list[idx].func_table;
				break;
			}
		}
	}

	if (NULL == func_list) {
		rsp_code = RC_NRC_REQUEST_OUT_OF_RANGE;
	} else {
		for (idx = 0; idx < RC_TYPE_NUM_MAX - 1; idx++) {
			if (RC_START_ROUTINE == func_list[idx].func_id) {
				record_length = func_list[idx].record_len;
				start_func = func_list[idx].func;
				break;
			}
		}
	}

	if (NULL == start_func) {
		rsp_code = RC_NRC_SUB_FUNC_NOT_SUPPORT;
	} else {
		if (rxdata->length - 4 != record_length) {
			rsp_code = RC_NRC_INCORRECT_MSG_LEN;
		} else {
			rsp_code = start_func(&rxdata->data[4], record_length);
		}
	}

	return rsp_code;
}

static uint8_t uds_stop_routine(uint16_t rid, pdu_t *rxdata, pdu_t *txdata)
{
        uint8_t rsp_code = 0xff;
        return rsp_code;
}

static uint8_t uds_request_routine_result(uint16_t rid, pdu_t *rxdata, pdu_t *txdata)
{
	uint8_t rsp_code = 0xff;
	return rsp_code;
}

void can_uds_routine_control(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id)
{
	uint8_t rsp_code = 0xFF;

	uint8_t sub_func = 0;
	uint16_t req_id = 0;
	// uint32_t d_idx = 0;

	do {
		if ((NULL == rx_data) || (NULL == tx_data)) {
			break;
		}

		if (rx_data->length < 4) {
			rsp_code = RC_NRC_INCORRECT_MSG_LEN;
			break;
		}

		sub_func = rx_data->data[1];
		req_id = rx_data->data[2];
		req_id <<= 8;
		req_id |= rx_data->data[3];
		if (req_id > ROUTINE_ID_MAX) {
			rsp_code = RC_NRC_REQUEST_OUT_OF_RANGE;
			break;
		}

		/* security access check? */
		;

		switch (sub_func) {
			case RC_START_ROUTINE:
				rsp_code = uds_start_routine(req_id, rx_data, tx_data);
				break;
			case RC_STOP_ROUTINE:
				rsp_code = uds_stop_routine(req_id, rx_data, tx_data);
				break;
			case RC_REQUEST_ROUTINE_RESULT:
				rsp_code = uds_request_routine_result(req_id, rx_data, tx_data);
				break;
			default:
				rsp_code = RC_NRC_SUB_FUNC_NOT_SUPPORT;
		}

		if (0 == rsp_code) {
			tx_data->data[0] = 0x71;
			tx_data->data[1] = sub_func;
			tx_data->data[2] = rx_data->data[2];
			tx_data->data[3] = rx_data->data[3];
			tx_data->length = 4;
		}

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		tx_data->data[0] = 0x7F;
		tx_data->data[1] = 0x31;
		tx_data->data[2] = rsp_code;
		tx_data->length = 3;
	}
}
