#ifndef UDS_SD_H
#define UDS_SD_H

typedef enum {
	UDS_SD_IDLE = 0,
	//UDS_SD_RX_BUF_ALLOC,
	//UDS_SD_RX_BUF_FILLING,
	//UDS_SD_WAIT_SERVICE,
	/* after this state,
	 * both rx&tx can't be used by other users. */
	UDS_SD_BUF_LOCK,
	UDS_SD_RX_DATA_READY,
	UDS_SD_SERVICE_IN_PROCESS,
	UDS_SD_SERVICE_DONE,
	//UDS_SD_TX_BUF_FLUSHING,
	UDS_SD_WAITING_TX_CONFIRM,
	UDS_SD_STATE_INVALID
} uds_sd_state_t;

typedef enum {
	//DEFAULT_SESSION = 0,
	UDS_ON_CAN,
	OBD_ON_CAN
} uds_type_t;

typedef enum {
	SESSION_ISO_SAE_RESERVED = 0,
	DEFAULT_SESSION,
	PROGRAMMING_SESSION,
	EXTENDED_DIAGNOSTIC_SESSION,
	SAFETY_SYSTEM_DIAGNOSTIC_SESSION,
	SESSION_INVALID_TYPE
} diag_session_type_t;

typedef enum {
	DIAG_PROTOCOL_NOT_USE = 0,
	DIAG_PROTOCOL_IN_USE
} diag_protocol_status_t;


typedef struct {
	uint8_t status;
	uint16_t size;
	uint16_t handled_size;
	uint16_t data_total_size;

	uint8_t *data;
} uds_buffer_t;
#define UDS_BUFFER_INIT(buf, data_ptr, buf_size) do {\
		(buf)->data = data_ptr;\
		(buf)->size = buf_size;\
		(buf)->handled_size = 0;\
		(buf)->data_total_size = 0;\
		(buf)->status = 0;\
	} while (0)

typedef struct {
	uint32_t p2server_max;
	uint32_t p2server_plus_max;
	uint32_t p3server_max;
} diag_protocol_timing_t;

typedef struct {
	uint8_t session_level;

	uint32_t p2server_max;
	uint32_t p2star_server_max;
} diag_session_t;

typedef struct {
	/* timer for processs. */
	uint32_t state;
	uint32_t state_timeout;

	/* timer for session. */
	uint32_t s3server_active;
	uint32_t s3server_timeout;

	uint32_t session_type;
	uint32_t security_level;

	pdu_t req_msg;
	pdu_t resp_msg;
} uds_runtime_t;

#define SERVICE_CNT_MAX         (20)

typedef struct {
	//uint32_t protocol_id;
	uint32_t sdu_id;

	/* started or closed? */
	uint32_t status;

	uint32_t priority;

	/* reserved. */
	uint32_t trans_type;

	diag_protocol_timing_t timing;

	uint8_t services[SERVICE_CNT_MAX];

	uds_buffer_t rx_buffer;
	uds_buffer_t tx_buffer;

	uds_runtime_t runtime;
	//uint32_t lock;
} uds_protocol_t;


typedef struct {
	uds_protocol_t *uds_protocol;

	//uint32_t rx_pdu_id;
	//uint32_t tx_pdu_id;
	diag_session_t *sessions;
	uint8_t nof_session;

	uint32_t addr_type;
	uint32_t tester_addr;

	uint32_t resp_pend_en;
	uint32_t resp_pend_cnt_max;

} uds_connect_t;


void uds_dispatch_init(void);
void uds_sd_state_main(void);
uint32_t uds_sd_xfer_capability(void);
int32_t uds_sd_txdata_copy(uint32_t sdu_id, pdu_t *pdu, uint32_t *remain_size);
int32_t uds_sd_rxdata_copy(uint32_t sdu_id, pdu_t *pdu, uint32_t *remain_size);
void uds_sd_confirmation(uint32_t sdu_id, uint32_t result);
void uds_sd_indication(uint32_t pud_id, uint32_t result);
int32_t uds_sd_malloc(uint32_t sdu_id, uint32_t size, uint32_t *remain_size);
diag_session_t *uds_findsession(uint8_t session_level);
int32_t diagnostic_change_session(uint32_t session_type);

#endif
