#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "radio_ctrl.h"
#include "uds.h"

#define CAN_UDS_LOCAL_BUFFER_SIZE	(0x408)


extern void *uds_get_uds_connect(uint32_t *cnt);


typedef struct {
	uint32_t inited;

	uint32_t nof_diagnostic;
	uds_connect_t *diagnostic_list;

	uds_connect_t *active_diagnostic;
} can_uds_global_t;


static uint8_t local_rx_buffer[CAN_UDS_LOCAL_BUFFER_SIZE];
static uint8_t local_tx_buffer[CAN_UDS_LOCAL_BUFFER_SIZE];
static can_uds_global_t can_uds;

static void uds_dispatch_reset(uds_protocol_t *protocol);
static int32_t diagnostic_service_supported(uds_protocol_t *protocol, uint32_t sid);
static void uds_services_handle(uds_connect_t *connect, uds_protocol_t *protocol, uint8_t sid);
static int32_t uds_protocol_activate(uds_connect_t *cur_diag, uds_protocol_t *protocol);
static void uds_service_rsp_done(uds_protocol_t *protocol, uint8_t sid);

void uds_dispatch_init(void)
{
	uint32_t i = 0;
	uds_runtime_t *runtime = NULL;
	uds_protocol_t *protocol = NULL;

	do {
		if (0 != can_uds.inited) {
			/* maybe there is a diagnostic is running.
			 * pls call uds_dispatch_deinit firstly. */
			break;
		}

		can_uds.diagnostic_list = (uds_connect_t *)uds_get_uds_connect(&can_uds.nof_diagnostic);
		if ((NULL == can_uds.diagnostic_list) || (0 == can_uds.nof_diagnostic)) {
			break;
		}

		for (; i < can_uds.nof_diagnostic; i++) {
			protocol = can_uds.diagnostic_list[i].uds_protocol;
			if (NULL != protocol) {
				/* not started. */
				protocol->status = DIAG_PROTOCOL_NOT_USE;

				/* register default buffer. */
				UDS_BUFFER_INIT(&protocol->rx_buffer, local_rx_buffer, CAN_UDS_LOCAL_BUFFER_SIZE);
				UDS_BUFFER_INIT(&protocol->tx_buffer, local_tx_buffer, CAN_UDS_LOCAL_BUFFER_SIZE);

				runtime = &protocol->runtime;
				/* state machine. */
				runtime->state = UDS_SD_IDLE;
				runtime->session_type = DEFAULT_SESSION;
				runtime->security_level = 0x0; /* locked. */
				runtime->s3server_active = 0;
			} else {
				break;
			}
		}

		if (1) {
			uint32_t pos = (uint32_t)local_rx_buffer;
			raw_writel(0xb30000, (pos >> 24) & 0xff);
			raw_writel(0xb30000, (pos >> 16) & 0xff);
			raw_writel(0xb30000, (pos >> 8) & 0xff);
			raw_writel(0xb30000, (pos >> 0) & 0xff);
		}

		can_uds.inited = 1;

	} while (0);

}

void uds_sd_send_response(uds_protocol_t *protocol)
{
	if (NULL != protocol) {
		uds_runtime_t *runtime = &protocol->runtime;
		pdu_t *pdu_info = &runtime->resp_msg;
		if ((0x7F == pdu_info->data[0]) && (0x78 == pdu_info->data[2])) {
			runtime->state_timeout = protocol->timing.p2server_plus_max;
		}

		cantp_transmit(protocol->sdu_id, &protocol->runtime.resp_msg);
	}
}

void uds_sd_state_main(void)
{
	uint32_t i = 0;

	uint32_t sid = 0;
	uds_protocol_t *protocol = NULL;
	uds_connect_t *cur_diag = NULL;
	uds_buffer_t *rxbuf = NULL;

	for (; i < can_uds.nof_diagnostic; i++) {
		cur_diag = &can_uds.diagnostic_list[i];
		if (NULL == cur_diag) {
			continue;
		}

		protocol = cur_diag->uds_protocol;
		if (0 == protocol->status) {
			continue;
		}
		rxbuf = &protocol->rx_buffer;

#if 0
		/* state */
		switch (protocol->state) {
			case UDS_SD_IDLE:
				break;
			case UDS_SD_BUF_LOCK:
				break;
			case UDS_SD_RX_DATA_READY:
				uds_service_dispatch();
				break;

			case UDS_SD_SERVICE_IN_PROCESS:
				/* continue! */
				break;

			case UDS_SD_WAITING_TX_CONFIRM:
				break;
			default:
				/* TODO: record error info. */
				break;
		}
#else
		if (UDS_SD_RX_DATA_READY == protocol->runtime.state) {
			/* parse msg, get sid. */
			sid = protocol->runtime.req_msg.data[0];

			/* check whether the uds protocol support the service.
			 * if not, send NCR.
			 * */
			if (0 == diagnostic_service_supported(protocol, sid)) {
				/* TODO: send NCR. */
				continue;
			}

			/* check session! */
			;

			/* check security! */
			;

			protocol->runtime.state = UDS_SD_SERVICE_IN_PROCESS;
			uds_services_handle(cur_diag, protocol, sid);
			/* reset buffer. */
			rxbuf->handled_size = 0;
			protocol->runtime.state = UDS_SD_SERVICE_DONE;
		}

		if (UDS_SD_SERVICE_DONE == protocol->runtime.state) {
			uds_sd_send_response(protocol);
			protocol->runtime.state = UDS_SD_IDLE;
		}
#endif
	}
}

static int32_t diagnostic_lookup(uds_connect_t **diag, uint32_t sdu_id)
{
	int32_t existed = 0, loopcnt = 0;

	uds_connect_t *cur_diag = NULL;
	for (; loopcnt < can_uds.nof_diagnostic; loopcnt++) {
		cur_diag = &can_uds.diagnostic_list[loopcnt];
		if (NULL != cur_diag) {
			if (sdu_id == cur_diag->uds_protocol->sdu_id) {
				existed = 1;
				*diag = cur_diag;
				break;
			}
		}
	}

	return existed;
}

int32_t uds_sd_malloc(uint32_t sdu_id, uint32_t size, uint32_t *remain_size)
{
	int32_t result = E_OK;

	uint32_t cpu_sts;
	//uint32_t index = 0;

	uds_protocol_t *protocol = NULL;
	uds_connect_t *cur_diag = NULL;
	uds_buffer_t *rxbuf = NULL;

	do {
		if (diagnostic_lookup(&cur_diag, sdu_id)) {
			protocol = cur_diag->uds_protocol;
		}
		if (NULL == protocol) {
			result = E_SYS;
			break;
		}

		if (DIAG_PROTOCOL_NOT_USE == protocol->status) {
			result = uds_protocol_activate(cur_diag, protocol);
			if (E_OK != result) {
				break;
			}
			/* register default buffer. */
			UDS_BUFFER_INIT(&protocol->rx_buffer, local_rx_buffer, \
					CAN_UDS_LOCAL_BUFFER_SIZE);
			UDS_BUFFER_INIT(&protocol->tx_buffer, local_tx_buffer, \
					CAN_UDS_LOCAL_BUFFER_SIZE);
		}

		cpu_sts = arc_lock_save();
		if (UDS_SD_IDLE != protocol->runtime.state) {
			result = E_DBUSY;
			arc_unlock_restore(cpu_sts);
			break;
		}
		protocol->runtime.state = UDS_SD_BUF_LOCK;
		arc_unlock_restore(cpu_sts);

		rxbuf = &protocol->rx_buffer;
		if (NULL == rxbuf->data) {
			result = E_SYS;
			break;
		}

		*remain_size = rxbuf->size - rxbuf->handled_size;
		if (size > *remain_size) {
			result = E_BOVR;
			break;
		}

		/* store message info. */
		//protocol->runtime.req_msg.length = size;
		//protocol->runtime.req_msg.data = &rxbuf->data[rxbuf->handled_size];

	} while (0);

	return result;
}

#if 0
static void trace_info(uint32_t value)
{
	static uint32_t idx = 0;
	raw_writel(0xa04000 + (idx++ << 2), value);
}
#endif

int32_t uds_sd_rxdata_copy(uint32_t sdu_id, pdu_t *pdu, uint32_t *remain_size)
{
	int32_t result = E_OK;

	//uint32_t index = 0;

	uds_protocol_t *protocol = NULL;
	uds_connect_t *cur_diag = NULL;
	uds_buffer_t *rxbuf = NULL;

	do {
		if (diagnostic_lookup(&cur_diag, sdu_id)) {
			protocol = cur_diag->uds_protocol;
		}
		if (NULL == protocol) {
			result = E_SYS;
			break;
		}

		if ((UDS_SD_BUF_LOCK != protocol->runtime.state)) {
			result = E_SYS;
			break;
		}

		rxbuf = &protocol->rx_buffer;
		if (NULL == rxbuf->data) {
			result = E_SYS;
			break;
		}

		*remain_size = rxbuf->size - rxbuf->handled_size;
		if (pdu->length > *remain_size) {
			result = E_BOVR;
			break;
		}
		memcpy(&rxbuf->data[rxbuf->handled_size], pdu->data, pdu->length);

		*remain_size -= pdu->length;
		rxbuf->handled_size += pdu->length;

	} while (0);

	return result;
}

uint32_t uds_sd_xfer_capability(void)
{
	return CAN_UDS_LOCAL_BUFFER_SIZE;
}

/* transport msg to dsd to process. */
void uds_sd_indication(uint32_t pud_id, uint32_t result)
{
	//uint32_t index = 0;
	//uint32_t flag = 0;

	uds_protocol_t *protocol = NULL;
	uds_connect_t *cur_diag = NULL;
	uds_buffer_t *rxbuf = NULL;
	uds_buffer_t *txbuf = NULL;

	raw_writel(0xb30000, 0x88);
	raw_writel(0xb30000, result);
	do {
		uint8_t sdu_id = 0; 
		if (diagnostic_lookup(&cur_diag, sdu_id)) {
			protocol = cur_diag->uds_protocol;
		}
		if (NULL == protocol) {
			result = E_SYS;
			break;
		}
		rxbuf = &protocol->rx_buffer;
		txbuf = &protocol->tx_buffer;

		if (UDS_SD_BUF_LOCK != protocol->runtime.state) {
			result = E_SYS;
			break;
		}

		if (N_RESULT_OK != result) {
			/* TODO: need to check whether the protocol is active? */

			/* release buffer, and restart session timeout. */
			protocol->rx_buffer.handled_size = 0;
			protocol->runtime.s3server_timeout = protocol->timing.p3server_max;
			protocol->runtime.s3server_active = 1;
			protocol->runtime.state = UDS_SD_IDLE;
			break;
		}

#if 0
		if (protocol->rx_buffer.handled_size != protocol->runtime.req_msg.length) {
			/* release buffer, and restart session timeout. */
			protocol->rx_buffer.handled_size = 0;
			protocol->runtime.s3server_timeout = protocol->timing.p3server_max;
			protocol->runtime.s3server_active = 1;
			protocol->runtime.state = UDS_SD_IDLE;
			break;
		}
#endif

		if (DIAG_PROTOCOL_NOT_USE == protocol->status) {
			result = uds_protocol_activate(cur_diag, protocol);
			if (E_OK != result) {
				break;
			}

			/* register default buffer. */
			UDS_BUFFER_INIT(rxbuf, local_rx_buffer, CAN_UDS_LOCAL_BUFFER_SIZE);
			UDS_BUFFER_INIT(txbuf, local_tx_buffer, CAN_UDS_LOCAL_BUFFER_SIZE);

		}

		if (DIAG_PROTOCOL_IN_USE == protocol->status) {
			protocol->runtime.state_timeout = protocol->timing.p2server_max;
			if (protocol->runtime.state_timeout > 0) {
				protocol->runtime.state_timeout -= 1;
			}

			/* response message buffer setting. */
			protocol->runtime.resp_msg.data = protocol->tx_buffer.data;
			//protocol->runtime.resp_msg.length = protocol->tx_buffer.handled_size;

			/* TODO: supress response cnt need to be store in runtime? */
			;

			protocol->runtime.req_msg.length = rxbuf->handled_size;
			protocol->runtime.req_msg.data = rxbuf->data;
			protocol->runtime.state = UDS_SD_RX_DATA_READY;
		}

	} while (0);

}


/* the lower layer copy data from uds buffer. */
int32_t uds_sd_txdata_copy(uint32_t sdu_id, pdu_t *pdu, uint32_t *remain_size)
{
	int32_t result = E_OK;

	//uint32_t index = 0;
	uint32_t offset = 0, unhandled_size = 0;

	uds_protocol_t *protocol = NULL;
	uds_connect_t *cur_diag = NULL;
	uds_runtime_t *runtime = NULL;

	do {
		if (diagnostic_lookup(&cur_diag, sdu_id)) {
			protocol = cur_diag->uds_protocol;
			runtime = &protocol->runtime;
		} else {
			result = E_SYS;
			break;
		}

		if (UDS_SD_SERVICE_DONE != protocol->runtime.state) {
			result = E_SYS;
			break;
		}

		/*
		unhandled_size = protocol->tx_buffer.size - protocol->tx_buffer.handled_size;
		offset = protocol->tx_buffer.handled_size;
		*/
		if (runtime->resp_msg.length <= protocol->tx_buffer.handled_size) {
			result = E_SYS;
			break;
		}

		unhandled_size = runtime->resp_msg.length - protocol->tx_buffer.handled_size;
		if (unhandled_size < pdu->length) {
			result = E_BOVR;
			break;
		}
		offset = protocol->tx_buffer.handled_size;

		memcpy(pdu->data, &runtime->resp_msg.data[offset], pdu->length);
		/* TODO: must use requst or response msg. */
		protocol->tx_buffer.handled_size += pdu->length;
		//*remain_size = protocol->tx_buffer.size - protocol->tx_buffer.handled_size;
		*remain_size = runtime->resp_msg.length - protocol->tx_buffer.handled_size;

	} while (0);

	return result;
}

void uds_sd_confirmation(uint32_t sdu_id, uint32_t result)
{
	/* TODO: check whether the msg has been transmited.
	 * if yes, then release tx buffer and confirmation to service.
	 * and restart session timer(s3server_max). */
	uint8_t *data = NULL;
	uds_protocol_t *protocol = NULL;
	uds_connect_t *cur_diag = NULL;
	uds_runtime_t *runtime = NULL;

	do {
		if (N_RESULT_OK != result) {
			/* TODO:  */
			break;
		}

		if (diagnostic_lookup(&cur_diag, sdu_id)) {
			protocol = cur_diag->uds_protocol;
			runtime = &protocol->runtime;
		} else {
			/* TODO:  */
			break;
		}

		protocol->tx_buffer.handled_size = 0;
		runtime->state = UDS_SD_IDLE;
		runtime->s3server_timeout = protocol->timing.p3server_max;

		/* TODO: comfirm to current service. */
		data = runtime->resp_msg.data;
		if ((NULL == data) || (runtime->resp_msg.length < 2)) {
			break;
		}

		if (0x7F != data[0]) {
			uint8_t sid = data[1] & 0x3F;
			uds_service_rsp_done(protocol, sid);
		}

	} while (0);
}

static int32_t diagnostic_service_supported(uds_protocol_t *protocol, uint32_t sid)
{
	int32_t supported = 0;
	uint32_t idx = 0;
	//uint8_t services[CNT_MAX];
	for (; idx < SERVICE_CNT_MAX; idx++) {
		if (sid == protocol->services[idx]) {
			supported = 1;
			break;
		}
	}
	return supported;
}

static void can_uds_ota_handshake(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id)
{
        do {
                if ((NULL == rxdata) || (NULL == txdata)) {
                        /* what happened? */
                        break;
                }

                /* filling response. */
                txdata->data = rxdata->data;
                txdata->length = 8;

                fmcw_radio_reboot_cause_set(ECU_REBOOT_CAN_OTA);

                chip_hw_mdelay(1);

                fmcw_radio_reset();

        } while (0);
}

static can_service_t uds_services[] = {
	CAN_UDS_SERVICE(CAN_UDS_DIAG_SESSION_CTRL, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_ECU_RESET, uds_ecu_reset, NULL),
	CAN_UDS_SERVICE(CAN_UDS_SECURITY_ACCESS, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_COMM_CTRL, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_TESTER_PRESENT, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_DTC_SETTING, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_CLEAR_DTC_INFO, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_READ_DTC_INFO, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_ROUTINE_CTRL, can_uds_routine_control, NULL),
	CAN_UDS_SERVICE(CAN_UDS_IOCTRL_BY_ID, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_READ_DATA_BY_ID, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_READ_MEM_BY_ADDR, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_READ_SCALE_DATA_BY_ID, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_READ_DATA_BY_PERIODID, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_WRITE_DATA_BY_ID, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_REQUEST_DOWNLOAD, can_uds_request_download, NULL),
	CAN_UDS_SERVICE(CAN_UDS_REQUEST_UPLOAD, NULL, NULL),
	CAN_UDS_SERVICE(CAN_UDS_TRANSFER_DATA, can_uds_transfer_data, NULL),
	CAN_UDS_SERVICE(CAN_UDS_REQUEST_TRANSFER_EXIT, can_uds_transfer_exit, NULL),
	CAN_UDS_SERVICE(CAN_UDS_REQUEST_FILE_TRANSFER, can_uds_ota_handshake, NULL),
};

static void uds_service_rsp_done(uds_protocol_t *protocol, uint8_t sid)
{
	if ((NULL != protocol) && diagnostic_service_supported(protocol, sid)) {
		uint32_t idx = 0;
		for (; idx < sizeof(uds_services) / sizeof(can_service_t); idx++) {
			if (sid == uds_services[idx].sid) {
				uds_services[idx].done(0, N_RESULT_OK);
				//uart_print(0x0A0D3030);
				break;
			}
		}
	}
}

static void uds_services_handle(uds_connect_t *connect, uds_protocol_t *protocol, uint8_t sid)
{
	if ((NULL != connect) && (NULL != protocol)) {
		/* look up service handler. */
		uint32_t idx = 0;
		service_handler handler = NULL;
		for (; idx < sizeof(uds_services) / sizeof(can_service_t); idx++) {
			if (sid == uds_services[idx].sid) {
				handler = uds_services[idx].service;
				//uart_print(0x0A0D4545);
				//uint8_t buf[1];
				//buf[0] = sid;
				//uart_write(buf, 1);
				//uart_print(0x0A0D4600 | (sid & 0xFF));
				break;
			}
		}

		if (handler) {
			uds_runtime_t *runtime = &protocol->runtime;
			pdu_t *rxdata = &runtime->req_msg;
			pdu_t *txdata = &runtime->resp_msg;
			/* call handler. */
			handler(rxdata, txdata, 0);
			raw_writel(0xb30000, sid);
			raw_writel(0xb30000, 0x8F);
		}
	}
}

int32_t diagnostic_change_session(uint32_t session_type)
{
	int32_t result = E_OK;

	//uint8_t old_session = 0;
	uds_protocol_t *protocol = NULL;
	uds_runtime_t *runtime = NULL;
	diag_session_t *session_ptr = NULL;

	do {
		if (NULL == can_uds.active_diagnostic) {
			result = E_SYS;
			break;
		}

		protocol = can_uds.active_diagnostic->uds_protocol;
		runtime = &protocol->runtime;

		/* TODO: security reset to relocked? */
		runtime->security_level = 0;

		//old_session = runtime->session_type;
		/* TODO: reset current session: buffer/services runtime data,
		 * such as, session control, communication control. */
		dcm_services_reset();
		uds_dispatch_reset(protocol);

		/* TODO: init new session, setup new session timing. */
		session_ptr = uds_findsession(session_type);
		if (NULL != session_ptr) {
			protocol->timing.p2server_max = session_ptr->p2server_max;
			protocol->timing.p2server_plus_max = session_ptr->p2star_server_max;
		}
		runtime->session_type = session_type;

	} while (0);


	return result;
}

static int32_t uds_protocol_activate(uds_connect_t *cur_diag, uds_protocol_t *protocol)
{
	int32_t result = E_OK;

	if (NULL == can_uds.active_diagnostic) {
		/* start diagnostic protocol. */
		protocol->status = DIAG_PROTOCOL_IN_USE;

		/* set session to default. */
		diagnostic_change_session(DEFAULT_SESSION);

		/* there is an issue:
		 * the current session is default, and there is
		 * a pending request message, if the request should
		 * be dealed in non-default session. how to do it?
		 * ignore the pending request message? or directly
		 * change session type to diagnostic protocol default
		 * session type(such as UDS_ON_CAN)?
		 * */
		;
		can_uds.active_diagnostic = cur_diag;

	} else {
		/*
		 * currently system has an active diagnostic,
		 * so compare their priority. if the new one has
		 * higher priority, then pending the current one,
		 * and start the new one. otherwise pending it, or
		 * ignore the request message?
		 * */
		if (protocol->priority > can_uds.active_diagnostic->uds_protocol->priority) {
			protocol->status = DIAG_PROTOCOL_IN_USE;

			/* set session to default. */
			diagnostic_change_session(DEFAULT_SESSION);

			//active_diagnostic->uds_prot.status = DIAG_PROTOCOL_PENDING;
			can_uds.active_diagnostic = cur_diag;
		} else {
			/* TODO: priority is too low! */
			result = E_SYS;
		}
	}

	return result;
}


diag_session_t *uds_findsession(uint8_t session_level)
{
	diag_session_t *session = NULL;
	if (NULL != can_uds.active_diagnostic) {
		uint8_t idx, session_cnt = can_uds.active_diagnostic->nof_session;
		diag_session_t *session_list = can_uds.active_diagnostic->sessions;
		if (session_list) {
			for (idx = 0; idx < session_cnt; idx++) {
				if (session_level == session_list->session_level) {
					session = session_list;
					break;
				} else {
					session_list++;
				}
			}
		}
	}
	return session;
}

static void uds_dispatch_reset(uds_protocol_t *protocol)
{
	uds_runtime_t *runtime = NULL;

	if (NULL != protocol) {
		runtime = &protocol->runtime;

		UDS_BUFFER_INIT(&protocol->rx_buffer, local_rx_buffer, CAN_UDS_LOCAL_BUFFER_SIZE);
		UDS_BUFFER_INIT(&protocol->tx_buffer, local_tx_buffer, CAN_UDS_LOCAL_BUFFER_SIZE);

		runtime->req_msg.data = NULL;
		runtime->req_msg.length = 0;

		runtime->resp_msg.data = NULL;
		runtime->resp_msg.length = 0;
	}
}
