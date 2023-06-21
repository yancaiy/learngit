#ifndef CAN_TP_H
#define CAN_TP_H

#define CANTP_PCI_SF		(0x00)
#define CANTP_PCI_FF		(0x10)
#define CANTP_PCI_CF		(0x20)
#define CANTP_PCI_FC		(0x30)

typedef enum {
	SINGLE_FRAME = 0,
	FIRST_FRAME,
	CONSECUTIVE_FRAME,
	FLOW_CONTROL_FRAME,
	FRAME_TYPE_INVALID
} cantp_frame_type_t;

typedef enum {
	CANTP_TX_IDLE = 0,
	CANTP_TX_WAIT_TRANSMIT,
	CANTP_TX_TRANSMITTING,
	CANTP_TX_WAIT_CONFIRMATION,

	CANTP_TX_WAIT_STMIN,
	CANTP_TX_WAIT_FC,
	CANTP_TX_INVALID_STATE
} cantp_tx_state_t;

typedef enum {
	CANTP_RX_IDLE = 0,
	/* allocate buffer from upper buffer list. */
	CANTP_RX_SF_ALLOC_BUF,
	CANTP_RX_FF_ALLOC_BUF,
	/* store msg on local buffer, waiting upper buffer. */
	CANTP_RX_SF_WAIT_BUF,
	/* store msg on local buffer, waiting upper buffer. */
	CANTP_RX_CF_WAIT_BUF,
	/* wait next consecutive frame. */
	CANTP_RX_WAIT_CF,
	CANTP_RX_STATE_INVALID
} cantp_rx_state_t;

typedef enum cantp_fc_status {
	CANTP_FS_CTS = 0,
	CANTP_FS_WAIT,
	CANTP_FS_OVERFLOW,
	CANTP_FS_INVALID
} cantp_fs_t;

#if 0
typedef struct {
	uint32_t sdu_id;

	uint32_t pdu_id;
	//uint32_t fc_pdu_id;
	uint32_t addr_xtd;
	uint32_t n_as;
	uint32_t n_bs;
	uint32_t n_cs;
	uint32_t connect_id;

	/*address mode: physical or functional. */
	uint32_t ta_type;
	uint32_t s_addr;
	uint32_t t_addr;

	uint32_t padding_active;
} cantp_tx_nsdu_t;

typedef struct {
	uint32_t sdu_id;

	uint32_t fc_pdu_id;
	uint32_t addr_xtd;
	uint32_t st_min;
	uint32_t bs;
	uint32_t wft_max;
	uint32_t n_ar;
	uint32_t n_br;
	uint32_t n_cr;
	uint32_t connect_id;

	/*address mode: physical or functional. */
	uint32_t ta_type;
	uint32_t s_addr;
	uint32_t t_addr;
	uint32_t padding_active;
} cantp_rx_nsdu_t;
typedef struct {
	//uint32_t xfer_type;
	uint32_t nof_rx_nsdu;
	uint32_t nof_tx_nsdu;

	cantp_rx_nsdu_t *rx_nsdu_list;
	cantp_tx_nsdu_t *tx_nsdu_list;
} cantp_nsdu_t;
#else

/* a diagnostic protocol map to a nsdu. */
typedef struct {
	uint32_t sdu_id;

	uint32_t tx_pdu_id;
	uint32_t rx_pdu_id;

	uint32_t addr_xtd;

	/* transmission. */
	uint32_t n_as;
	uint32_t n_bs;
	uint32_t n_cs;

	/* receiving. */
	uint32_t st_min;
	uint32_t bs;
	uint32_t wft_max;
	uint32_t n_ar;
	uint32_t n_br;
	uint32_t n_cr;

	/*address mode: physical or functional. */
	uint32_t ta_type;
	uint32_t s_addr;
	uint32_t t_addr;

	uint32_t padding_active;

	uint32_t connect_id;

	/* TODO: maybe need move to other structure! */
	uint32_t frame_length;
} cantp_nsdu_t;
#endif

typedef enum can_net_result {
	N_RESULT_OK = 0,
	N_RESULT_TIMEOUT_A,
	N_RESULT_TIMEOUT_BS,
	N_RESULT_TIMEOUT_CR,
	N_RESULT_WRONG_SN,
	N_RESULT_INVALID_FS,
	N_RESULT_UNEXP_PDU,
	N_RESULT_WFT_OVRN,
	N_RESULT_BUFFER_OVFLW,
	N_RESULT_ERROR
} net_result_t;



typedef struct can_message {
	uint8_t data[64];
	uint8_t length;
	uint8_t index;
} can_msg_t;

typedef struct {
	uint32_t next_fc_cnt;

	uint32_t st_min;
	uint32_t bs;
	uint32_t nas;
	uint32_t nar;
	uint32_t wft_max_cnt;

	uint8_t tx_sn;
	uint8_t rx_sn;

	uint32_t upper_buf_size;
} cantp_session_t;

typedef struct {
	uint32_t sdu_id;

	/* while xfer, store for can_if. */
	uint32_t pdu_id;

	/* xfer_proc_ctrl; */
	uint8_t *msg_data;
	uint32_t xfer_total_cnt;
	uint32_t xfer_cnt;
	can_msg_t buffer;

	/* rx_wait, rx_processing, tx_wait, tx_processing. */
	uint32_t state;
	uint32_t state_timeout;

	cantp_session_t xfer_ctrl;

	/* write protect. */
	//uint32_t lock;
} cantp_channel_t;

typedef struct {
	uint32_t ctrl_id;

	/* two physical channel and two function channel. */
	cantp_channel_t tx_chn;
	cantp_channel_t rx_chn;
} cantp_connect_t;


void cantp_init(void);

int32_t cantp_transmit(uint32_t sdu_id, pdu_t *pdu);
void cantp_confirmation(uint32_t pdu_id);
void cantp_indication(uint32_t pdu_id, pdu_t *pdu);

void cantp_state_main(void);

#endif
