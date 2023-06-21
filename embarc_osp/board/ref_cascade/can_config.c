#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "dev_can.h"
#include "can_hal.h"
#include "can_if.h"
#include "can_tp.h"
#include "can_baud_def.h"

#include "uds_sd.h"
#include "services.h"

/* one of can driver config:
 * @line_id, the hardware interrupt signal logical id.
 * @bitmap, indicate which can internal interrupt signals will be assigned to @line_id.
 * */
typedef struct {
        uint8_t line_id;
        uint8_t activition;
        uint32_t bitmap;
} can_intmap_t;


/* baud rate parameters. */
typedef struct {
        uint32_t rate;
        uint32_t fd_rate;
        can_baud_t baud_param;
        can_baud_t fd_baud_param;
#if 0
        uint32_t prop_reg;
        uint32_t seg1;
        uint32_t seg2;
        uint32_t jump_width;
#endif
} can_raw_baud_cfg_t;

typedef struct {
        uint32_t ref_clock;
        can_raw_baud_cfg_t baud[4];
} can_baud_cfg_t;

/* the configuration of the can controller.
 * @activiation: indicate if the controller is used or not.
 * @nof_intmap: number of the intmap.
 * @intmap: describe how to map the hardware interrupt signals in controller to the signals
 *  connect to CPU.
 * @baud_cfg: the congiguration of the baud rate.
 * @nof_baud_cfg: number of the baud rate configuration.
 * @def_baud: the default baud rate.
 * */
typedef struct {
        uint8_t ctrl_id;

        uint8_t activiation;

        uint8_t fd_en;
        uint8_t brs;

        /* RX/TX DFS. */
        uint8_t rx_dfs;
        uint8_t tx_dfs;

        uint8_t nof_intmap;
        uint8_t nof_baud_cfg;

        /* using internal or external reference clock?
         * 0->using SoC internal reference clock.
         * 1->using ouside input clock.
         * */
        uint8_t ref_clock_sel;

        uint32_t def_baud;
        can_baud_cfg_t *baud_cfg;

        can_intmap_t *intmap;
} can_controller_cfg_t;
#if 0
/* CAN configuration:
 * @nof_ctrl: number of can controller.
 * @txchn2ctrl: the mapping of the logic transmitting channel id to hardware controller id.
 * @ctrl2rxchn: the mapping of the hardware controller id to logic receiving channel id. if
 *  necessary, one controller id can map to multiple logic receiving channels based on CAN ID.
 * */
typedef struct {
        uint8_t nof_ctrl;

        /* logic tx channel. */
        uint8_t nof_tx_chn;

        can_controller_cfg_t *ctrl_cfg;

        //uint8_t *txchn2txbuf;
        uint8_t *txchn2ctrl;
        uint8_t *ctrl2rxchn;
} can_config_t;

/***************** can driver. *******************/
static can_baud_cfg_t can_baud[] = {
{
	.ref_clock = CLOCK_40MHZ,
	.baud = {
		{.rate = CAN_BAUDRATE_100KBPS, .fd_rate = CAN_BAUDRATE_200KBPS, CAN_BAUD_DESC_40MHZ_100KBPS, CAN_BAUD_DESC_40MHZ_200KBPS},
		{.rate = CAN_BAUDRATE_200KBPS, .fd_rate = CAN_BAUDRATE_500KBPS, CAN_BAUD_DESC_40MHZ_200KBPS, CAN_BAUD_DESC_40MHZ_500KBPS},
		{.rate = CAN_BAUDRATE_500KBPS, .fd_rate = CAN_BAUDRATE_1MBPS, CAN_BAUD_DESC_40MHZ_500KBPS, CAN_BAUD_DESC_40MHZ_1MBPS},
	},
},

{
	.ref_clock = CLOCK_50MHZ,
	.baud = {
		{.rate = CAN_BAUDRATE_100KBPS, .fd_rate = CAN_BAUDRATE_200KBPS, CAN_BAUD_DESC_50MHZ_100KBPS, CAN_BAUD_DESC_50MHZ_200KBPS},
		{.rate = CAN_BAUDRATE_200KBPS, .fd_rate = CAN_BAUDRATE_500KBPS, CAN_BAUD_DESC_50MHZ_200KBPS, CAN_BAUD_DESC_50MHZ_500KBPS},
		{.rate = CAN_BAUDRATE_500KBPS, .fd_rate = CAN_BAUDRATE_1MBPS, CAN_BAUD_DESC_50MHZ_500KBPS, CAN_BAUD_DESC_50MHZ_1MBPS},
	},
},

{
	.ref_clock = CLOCK_100MHZ,
	.baud = {
		{.rate = CAN_BAUDRATE_100KBPS, .fd_rate = CAN_BAUDRATE_200KBPS, CAN_BAUD_DESC_100MHZ_100KBPS, CAN_BAUD_DESC_100MHZ_200KBPS},
		{.rate = CAN_BAUDRATE_200KBPS, .fd_rate = CAN_BAUDRATE_500KBPS, CAN_BAUD_DESC_100MHZ_200KBPS, CAN_BAUD_DESC_100MHZ_500KBPS},
		{.rate = CAN_BAUDRATE_500KBPS, .fd_rate = CAN_BAUDRATE_1MBPS, CAN_BAUD_DESC_100MHZ_500KBPS, CAN_BAUD_DESC_100MHZ_1MBPS},
	},
}
};

static can_intmap_t intmap[] = {
{
	/* interrupt signals class1: new message come. */
	.line_id = 0,

	/* buffer mode:  */
	.bitmap = CAN_INT_RX_NEW_MESSAGE,

	/* fifo mode: */
	//.mask = CAN_INT_RX_FIFO_FULL | CAN_INT_RX_FIFO_REAL_FULL,
},

{
	/* interrupt signals class2: transmission done. */
	.line_id = 1,

	/* buffer mode:  */
	.bitmap = (CAN_INT_TX_COMPLISHED | CAN_INT_TX_CANCEL_FINISHED),

	/* fifo mode:  */
	//.mask = (CAN_INT_TX_FIFO_EMPTY | CAN_INT_TX_CANCEL_FINISHED),
	//CAN_INT_TX_EVENT_FIFO_EMPTY
},

{
	/* interrupt signals class3: warning! */
	.line_id = 2,
	.bitmap = (CAN_INT_WARNING | \
		 CAN_INT_TIMESTAMP_WRAP_AROUND | \
		 CAN_INT_RX_FIFO_LOST | \
		 CAN_INT_TX_FIFO_LOST)
#if 0
		/* unused interrupt signals! */
		//CAN_INT_RX_FIFO_EMPTY
		//CAN_INT_TX_FIFO_REAL_FULL
		//CAN_INT_TX_FIFO_FULL
		//CAN_INT_TX_EVENT_FIFO_REAL_FULL
		//CAN_INT_TX_EVENT_FIFO_FULL
		//CAN_INT_TIMEOUT_OCCURED
		//CAN_INT_TX_EVENT_FIFO_LOST
#endif
},

{
	/* interrupt signals class4: error happened! */
	.line_id = 3,
	.bitmap = (CAN_INT_ACCESS_PROTECT_REG | \
		CAN_INT_PROTOCOL_ERR | \
		CAN_INT_BUS_OFF | \
		CAN_INT_ERR_PASSIVE | \
		CAN_INT_ERR_LOGGING_OVERFLOW | \
		CAN_INT_BIT_ERR_UNCORRECTED | \
		CAN_INT_BIT_ERR_CORRECTED | \
		CAN_INT_RAM_ACCESS_FAIL)
}
};

static can_controller_cfg_t can_ctrl_cfg[2] = {
{
	.ctrl_id = 0,
	.activiation = 1,
	.fd_en = 0,
	.brs = 0,

	.rx_dfs = 8,
	.tx_dfs = 8,
	.nof_intmap = sizeof(intmap) / sizeof(can_intmap_t),
	.nof_baud_cfg = sizeof(can_baud) / sizeof(can_baud_cfg_t),

	.ref_clock_sel = 0,
	.def_baud = CAN_BAUDRATE_500KBPS,
	.baud_cfg = can_baud,
	.intmap = intmap
},

{
	.ctrl_id = 1,
	.activiation = 0,
	.fd_en = 0,
	.brs = 0,

	.rx_dfs = 8,
	.tx_dfs = 8,
	.nof_intmap = sizeof(intmap) / sizeof(can_intmap_t),
	.nof_baud_cfg = sizeof(can_baud) / sizeof(can_baud_cfg_t),

	.ref_clock_sel = 0,
	.def_baud = CAN_BAUDRATE_500KBPS,
	.baud_cfg = can_baud,
	.intmap = intmap
}
};

static uint8_t txchn2ctrl[] = {0, 1};
static uint8_t ctrl2rxchn[] = {0, 1};
static can_config_t can_cfg = {
	.nof_ctrl = 2,
	.nof_tx_chn = 2,

	.txchn2ctrl = txchn2ctrl,
	.ctrl2rxchn = ctrl2rxchn,

	.ctrl_cfg = can_ctrl_cfg,
};

void *can_config_get(void)
{
	return (void *)&can_cfg;
}
#endif


/****************** can interface. **********************/
static canif_txpdu_cfg_t tx_pdu_list[] = {
	{
		.pdu_id = 0,
		.can_id = 0x11,
		.id_type = CAN_ID_STD,
		.dlc = 8,
		.bus_type = CAN_BUS_BASIC,
		.hw_chn_id = 0
	},
	{
		.pdu_id = 1,
		.can_id = 0x21,
		.id_type = CAN_ID_STD,
		.dlc = 8,
		.bus_type = CAN_BUS_BASIC,
		.hw_chn_id = 1
	},
};

static canif_rxpdu_cfg_t rx_pdu_list[] = {
	{
		.pdu_id = 0,
		.id_range = {0x0, 0xFFFFFFFF},
		.id_type = CAN_ID_STD,
		.dlc = 8,
		.bus_type = CAN_BUS_BASIC,
		.rx_indication_id = 0,
		.hw_chn_id = 0
	},
	{
		.pdu_id = 1,
		.id_range = {0x30000, 0x50000},
		.id_type = CAN_ID_STD,
		.dlc = 8,
		.bus_type = CAN_BUS_BASIC,
		.rx_indication_id = 1,
		.hw_chn_id = 0
	},
	{
		.pdu_id = 2,
		.id_range = {0x50, 0x60},
		.id_type = CAN_ID_STD,
		.dlc = 8,
		.bus_type = CAN_BUS_BASIC,
		.hw_chn_id = 0
	},
	{
		.pdu_id = 3,
		.id_range = {0x30, 0x40},
		.id_type = CAN_ID_STD,
		.dlc = 8,
		.bus_type = CAN_BUS_BASIC,
		.hw_chn_id = 1
	}
};

void *canif_get_rxpdu_cfg(uint32_t *cnt)
{
	*cnt = sizeof(rx_pdu_list) / sizeof(canif_rxpdu_cfg_t);
	return (void *)&rx_pdu_list;
}

void *canif_get_txpdu_cfg(uint32_t *cnt)
{
	*cnt = sizeof(tx_pdu_list) / sizeof(canif_txpdu_cfg_t);
	return (void *)&tx_pdu_list;
}


/********************** can transport layer. **************************/
#if 0
static cantp_tx_nsdu_t tx_nsdu_list[] = {
	{
		.sdu_id = 0,
		.pdu_id = 0,
		.fc_pdu_id = 0,
		.addr_xtd = 0,
		.n_as = 0x100,
		.n_bs = 0x100,
		.n_cs = 0x100,
		.chn_id = 0,

		.ta_type = 0,
		.s_addr = 0x211,
		.t_addr = 0x112,

		.padding_active = 0x55,
	},
};

static cantp_rx_nsdu_t rx_nsdu_list[] = {
	{
		.sdu_id = 1,
		.fc_pdu_id = 0,
		.addr_xtd = 0,
		.st_min = 0,
		.bs = 0xff,
		.wft_max = 0xff,
		.n_ar = 0x100,
		.n_br = 0x100,
		.n_cr = 0x100,
		.chn_id = 0,

		.ta_type = 0,
		.s_addr = 0,
		.t_addr = 0,
		.padding_active = 0xaa,
	},
};

void *cantp_get_txnsdu(uint32_t *cnt)
{
	*cnt = sizeof(tx_nsdu_list) / sizeof(cantp_tx_nsdu_t);

	return (void *)&tx_nsdu_list[0];
}
void *cantp_get_rxnsdu(uint32_t *cnt)
{
	*cnt = sizeof(rx_nsdu_list) / sizeof(cantp_rx_nsdu_t);

	return (void *)&rx_nsdu_list[0];
}

#else

static cantp_nsdu_t nsdu_list[] = {
{
	.sdu_id = 0,
	.tx_pdu_id = 0,
	.addr_xtd = 0,

	/* transmission. */
	.n_as = 0x100,
	.n_bs = 0x100,
	.n_cs = 0x100,

	/* receiving. */
	.st_min = 0,
	.bs = 0xff,
	.wft_max = 0xff,
	.n_ar = 0xffff,
	.n_br = 0xffff,
	.n_cr = 0xffff,

	/*address mode: physical or functional. */
	.ta_type = 0,
	.s_addr = 0,
	.t_addr = 0,

	.padding_active = 1,

	.connect_id = 0,

	.frame_length = 8,
},

{
	.sdu_id = 1,
	.tx_pdu_id = 1,
	.addr_xtd = 0,

	/* transmission. */
	.n_as = 0x100,
	.n_bs = 0x100,
	.n_cs = 0x100,

	/* receiving. */
	.st_min = 0,
	.bs = 0xff,
	.wft_max = 0xff,
	.n_ar = 0xffff,
	.n_br = 0xffff,
	.n_cr = 0xffff,

	/*address mode: physical or functional. */
	.ta_type = 0,
	.s_addr = 0,
	.t_addr = 0,

	.padding_active = 1,

	.connect_id = 0,

	.frame_length = 8,
},
};

void *cantp_get_nsdu(uint32_t *cnt)
{
	*cnt = sizeof(nsdu_list) / sizeof(cantp_nsdu_t);

	return (void *)&nsdu_list[0];
}
#endif


/********************* can uds protocol **********************/
static uds_protocol_t diag_protocol_list[] = {
{
	.sdu_id = 0,
	.status = 0,
	.priority = 1,
	.trans_type = 0,
	.timing = {
		.p2server_max = 0x100,
		.p2server_plus_max = 0x110,
		.p3server_max = 0x200
	},
	.services = {
	        CAN_UDS_DIAG_SESSION_CTRL,
	        CAN_UDS_ECU_RESET,
	        CAN_UDS_SECURITY_ACCESS,
	        CAN_UDS_COMM_CTRL,
	        CAN_UDS_TESTER_PRESENT,
	        CAN_UDS_DTC_SETTING,
	        CAN_UDS_CLEAR_DTC_INFO,
	        CAN_UDS_READ_DTC_INFO,
	        CAN_UDS_ROUTINE_CTRL,
	        CAN_UDS_IOCTRL_BY_ID,
	        CAN_UDS_READ_DATA_BY_ID,
	        CAN_UDS_READ_MEM_BY_ADDR,
	        CAN_UDS_READ_SCALE_DATA_BY_ID,
	        CAN_UDS_READ_DATA_BY_PERIODID,
	        CAN_UDS_WRITE_DATA_BY_ID,
	        CAN_UDS_REQUEST_DOWNLOAD,
	        CAN_UDS_REQUEST_UPLOAD,
	        CAN_UDS_TRANSFER_DATA,
	        CAN_UDS_REQUEST_TRANSFER_EXIT,
	        CAN_UDS_REQUEST_FILE_TRANSFER
	}
}
};

static diag_session_t uds_session_list[] = {
	{
		.session_level = DEFAULT_SESSION,
		.p2server_max = 500,
		.p2star_server_max = 1000
	},
	{
		.session_level = PROGRAMMING_SESSION,
		.p2server_max = 500,
		.p2star_server_max = 1000
	},
	{
		.session_level = EXTENDED_DIAGNOSTIC_SESSION,
		.p2server_max = 500,
		.p2star_server_max = 1000
	},
	/*
	{
		.session_level = SAFETY_SYSTEM_DIAGNOSTIC_SESSION,
		.p2server_max = 500,
		.p2star_server_max = 1000
	}
	*/
};

static uds_connect_t uds_con_list[] = {
{
	.uds_protocol = diag_protocol_list,
	.sessions = uds_session_list,
	.nof_session = sizeof(uds_session_list) / sizeof(diag_session_t),
	.addr_type = 0,
	.tester_addr = 0x1234,
	.resp_pend_en = 0,
	.resp_pend_cnt_max = 0
}
};

void *uds_get_uds_connect(uint32_t *cnt)
{
	*cnt = sizeof(uds_con_list) / sizeof(uds_connect_t);
	return (void *)&uds_con_list[0];
}
