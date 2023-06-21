#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "radio_ctrl.h"


typedef enum {
	ECU_HARD_RESET = 1,
	ECU_KEY_OFFON_RESET,
	ECU_SW_RESET,
	ECU_EN_RAPID_PWR_SHUTDOWN,
	ECU_DIS_RAPID_PWR_SHUTDOWN,
	ECU_INVIALD_RESET_TYPE
} ecu_reset_t;


/*
 * if ECUReset service need to response positive message to sendor,
 * then it send the response message, after wait confirmation, it
 * reset itself. if don't need to response positive message, immediately reset.
 * */
void uds_ecu_reset(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id)
{
	uint8_t reset_type;
	uint8_t rsp_code = 0xFF;

	do {
		if ((NULL == rx_data) || (NULL == tx_data)) {
			break;
		}

		if (2 != rx_data->length) {
			rsp_code = 0x13;
			break;
		}

		reset_type = rx_data->data[1];
		switch (reset_type) {
			case ECU_HARD_RESET:
				/* TODO: not support! */
				break;

			case ECU_SW_RESET:
				/* after response has been transfered successful? */
			        fmcw_radio_reboot_cause_set(ECU_REBOOT_NORMAL);
                                rsp_code = 0;

                                chip_hw_udelay(1000);
                                fmcw_radio_reset();
				break;

			case ECU_KEY_OFFON_RESET:
				/* whether need to achieve this? */
			case ECU_EN_RAPID_PWR_SHUTDOWN:
			case ECU_DIS_RAPID_PWR_SHUTDOWN:
			default:
				/* send NCR: sub-function not supported. */
				rsp_code = 0x12;
		}

		tx_data->data[0] = 0x51;
		tx_data->data[1] = 3;
		tx_data->data[2] = 0;
		tx_data->length = 3;

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		tx_data->data[0] = 0x7F;
		tx_data->data[1] = 0x11;
		tx_data->data[2] = rsp_code;
		tx_data->length = 3;
	}
}


typedef struct {
	uint8_t status;
	uint8_t session_type;
	uint32_t txpdu_id;
	pdu_t *rxdata;
	pdu_t *txdata;
} uds_dsc_runtime_t;
static uds_dsc_runtime_t session_runtime;
void uds_diagnostic_control(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id)
{
	uint8_t rsp_code = 0xFF;
	uint8_t sub_func = 0;

	diag_session_t *session = NULL;

	do {
		if ((NULL == rx_data) || (NULL == tx_data)) {
			break;
		}

		if (2 != rx_data->length) {
			rsp_code = 0x13;
			break;
		}

		sub_func = rx_data->data[1];
		session = uds_findsession(sub_func);
		if (NULL == session) {
			rsp_code = 0x12;
			break;
		}

		/* core here: store message, and waiting response confirmation. */
		session_runtime.status = 1;
		session_runtime.session_type = sub_func;
		session_runtime.txpdu_id = tx_pdu_id;
		session_runtime.rxdata = rx_data;
		session_runtime.txdata = tx_data;

		tx_data->data[0] = 0x50;
		tx_data->data[1] = sub_func;
		tx_data->data[2] = session->p2server_max >> 8;
		tx_data->data[3] = session->p2server_max & 0xFF;
		tx_data->data[4] = session->p2star_server_max >> 8;
		tx_data->data[5] = session->p2star_server_max;
		tx_data->length = 6;

		rsp_code = 0;

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		tx_data->data[0] = 0x7F;
		tx_data->data[1] = 0x10;
		tx_data->data[2] = rsp_code;
		tx_data->length = 3;
	}
}

void uds_diagnostic_control_rsp_done(uint8_t tx_pdu_id, uint32_t result)
{
	/* TODO: reset active protocol session, and switch to new session. */
	if (session_runtime.status) {
		if (tx_pdu_id == session_runtime.txpdu_id) {
			/* TODO:  other thing need to do? */
			;

			diagnostic_change_session(session_runtime.session_type);
		}

		session_runtime.status = 0;
	}
}


typedef enum {
	COMM_CTRL_ENRX_ENTX = 0,
	COMM_CTRL_ENRX_DISTX,
	COMM_CTRL_DISRX_ENTX,
	COMM_CTRL_DISRX_DISTX,
	COMM_CTRL_ENRX_DISTX_EAI,
	COMM_CTRL_ENRX_ENTX_EAI,
	COMM_CTRL_INVALID_TYPE
} comm_ctrl_type_t;

typedef enum {
	COMM_NCM = 1,
	COMM_NMCM,
	COMM_NCM_NMCM
} comm_type_t;

typedef struct {
	uint8_t status;
	uint8_t sub_func;
	uint8_t com_type;
	uint8_t net_ctrl;
	uint32_t txpdu_id;
	pdu_t *rxdata;
	pdu_t *txdata;
} uds_dcc_runtime_t;
static uds_dcc_runtime_t com_ctrl_runtime;

/* The purpose of this service is to switch on/off the transmission
 * and/or the reception of certain messages of a server. */
void uds_communication_control(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id)
{
	uint8_t rsp_code = 0xFF;

	uint8_t sub_func = 0;
	uint8_t com_type = 0, net_ctrl = 0;
	uint16_t node_id = 0;

	do {
		if ((NULL == rx_data) || (NULL == tx_data)) {
			break;
		}

		if (rx_data->length < 3) {
			rsp_code = 0x13;
			break;
		}

		sub_func = rx_data->data[1];
		com_type = rx_data->data[2] & 0x3;
		net_ctrl = rx_data->data[2] >> 4;

		switch (sub_func) {
			case COMM_CTRL_ENRX_ENTX:
			case COMM_CTRL_ENRX_DISTX:
			case COMM_CTRL_DISRX_ENTX:
			case COMM_CTRL_DISRX_DISTX:
				rsp_code = 0;
				break;

			case COMM_CTRL_ENRX_DISTX_EAI:
			case COMM_CTRL_ENRX_ENTX_EAI:
				if (5 != rx_data->length) {
					rsp_code = 0x13;
					break;
				} else {
					node_id = rx_data->data[3];
					node_id <<= 8;
					node_id |= rx_data->data[4];
				}
				break;

			default:
				rsp_code = 0x12;
		}

		if (0 != rsp_code) {
			break;
		}
		if ((0 == com_type) || (com_type > COMM_NCM_NMCM)) {
			rsp_code = 0x31;
			break;
		}

		/* TODO: maybe need to check the control of each communication type,
		 * com_type: sub_func.
		 * */

		/* core here: store message, and waiting response confirmation. */
		com_ctrl_runtime.status = 1;
		com_ctrl_runtime.sub_func = sub_func;
		com_ctrl_runtime.com_type = com_type;
		com_ctrl_runtime.net_ctrl = net_ctrl;
		com_ctrl_runtime.txpdu_id = tx_pdu_id;
		com_ctrl_runtime.rxdata = rx_data;
		com_ctrl_runtime.txdata = tx_data;

		if (0 == rsp_code) {
			tx_data->data[0] = 0x68;
			tx_data->data[1] = sub_func;
			tx_data->length = 2;
		}

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		tx_data->data[0] = 0x7F;
		tx_data->data[1] = 0x28;
		tx_data->data[2] = rsp_code;
		tx_data->length = 3;
	}
}

void uds_communication_control_rsp_done(uint8_t tx_pdu_id, uint32_t result)
{
	/* TODO:  switch to new mode. */
}


typedef enum {
	UDS_DTC_ON = 1,
	UDS_DTC_OFF
} uds_dtc_setting_type_t;
void uds_control_dtc_setting(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id)
{
	uint8_t rsp_code = 0xFF;

	uint8_t sub_func = 0;
	//uint8_t *record = NULL;

	do {
		if ((NULL == rx_data) || (NULL == tx_data)) {
			break;
		}

		if (rx_data->length < 2) {
			rsp_code = 0x13;
			break;
		}

		sub_func = rx_data->data[1];
		if (rx_data->length > 2) {
			//record = &rx_data->data[2];
		}

		switch (sub_func) {
			case UDS_DTC_ON:
				/* TODO:  */
				break;

			case UDS_DTC_OFF:
				/* TODO:  */
				break;

			default:
				rsp_code = 0x12;
		}

		if (0 == rsp_code) {
			tx_data->data[0] = 0xc5;
			tx_data->data[1] = sub_func;
			tx_data->length = 2;
		}

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		tx_data->data[0] = 0x7F;
		tx_data->data[1] = 0x85;
		tx_data->data[2] = rsp_code;
		tx_data->length = 3;
	}
}

void dcm_services_reset(void)
{
	/* TODO: */
}
