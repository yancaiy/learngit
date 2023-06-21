#ifndef SERVICES_H
#define SERVICES_H

#define CAN_UDS_DIAG_SESSION_CTRL	(0x10)
#define CAN_UDS_ECU_RESET		(0x11)
#define CAN_UDS_SECURITY_ACCESS		(0x27)
#define CAN_UDS_COMM_CTRL		(0x28)
#define CAN_UDS_TESTER_PRESENT		(0x3E)
#define CAN_UDS_DTC_SETTING		(0x85)
#define CAN_UDS_CLEAR_DTC_INFO		(0x14)
#define CAN_UDS_READ_DTC_INFO		(0x19)
#define CAN_UDS_ROUTINE_CTRL		(0x31)
#define CAN_UDS_IOCTRL_BY_ID		(0x2F)
#define CAN_UDS_READ_DATA_BY_ID		(0x22)
#define CAN_UDS_READ_MEM_BY_ADDR	(0x23)
#define CAN_UDS_READ_SCALE_DATA_BY_ID	(0x24)
#define CAN_UDS_READ_DATA_BY_PERIODID	(0x2A)
#define CAN_UDS_WRITE_DATA_BY_ID	(0x2E)

#define CAN_UDS_REQUEST_DOWNLOAD	(0x34)
#define CAN_UDS_REQUEST_UPLOAD		(0x35)
#define CAN_UDS_TRANSFER_DATA		(0x36)
#define CAN_UDS_REQUEST_TRANSFER_EXIT	(0x37)
#define CAN_UDS_REQUEST_FILE_TRANSFER	(0x38)


typedef void (*service_handler)(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
typedef void (*service_response_done)(uint8_t reserved, uint32_t result);
typedef struct {
	uint8_t sid;
	service_handler service;
	service_response_done done;
} can_service_t;


#define CAN_UDS_SERVICE(id, func, resp_func)	{.sid = id, .service = func, .done = resp_func}


void uds_ecu_reset(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id);
void can_uds_request_download(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id);
void can_uds_transfer_data(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id);
void can_uds_transfer_exit(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id);

void can_uds_routine_control(pdu_t *rx_data, pdu_t *tx_data, uint32_t tx_pdu_id);
void set_flash_addr(uint32_t addr);

#endif
