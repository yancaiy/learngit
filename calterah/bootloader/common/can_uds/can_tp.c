#include <string.h>
#include "embARC_toolchain.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_error.h"
//#include "arc_exception.h"

//#include "can_bus_config.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"

#include "can_config.h"

typedef struct {
	uint32_t sch_period;
} cantp_cfg_t;


#define CAN_BUS_CONTROLLER_CNT	(2)
typedef struct {
	uint32_t inited;

	cantp_cfg_t cfg;
	cantp_connect_t connect[CAN_BUS_CONTROLLER_CNT];

	uint32_t nof_nsdu;
	cantp_nsdu_t *nsdu_list;
} cantp_global_t;


static cantp_global_t cantp;


/************** local functions description *****************/
static void cantp_channel_init(cantp_channel_t *chn);
static uint32_t cantp_get_ff_datalen(uint32_t addr_xtd, pdu_t *pdu);
static int32_t cantp_fill_frame(cantp_nsdu_t *tx_nsdu, cantp_channel_t *tx_chn);
static int32_t cantp_send_frame(cantp_channel_t *tx_chn, cantp_nsdu_t *tx_nsdu);
static void cantp_send_fc(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, uint32_t status);
static int32_t cantp_fc_handle(cantp_channel_t *tx_chn, cantp_nsdu_t *tx_nsdu, pdu_t *pdu);
static int32_t cantp_ff_handle(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, pdu_t *pdu);
static int32_t cantp_cf_handle(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, pdu_t *pdu);
static int32_t cantp_sf_handle(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, pdu_t *pdu);

void cantp_init(void)
{
	uint32_t cnt = 0;

	if (0 == cantp.inited) {
		cantp.cfg.sch_period = 100;

		cantp.nsdu_list = (cantp_nsdu_t *)cantp_get_nsdu(&cnt);
		if (cantp.nsdu_list) {
			cantp.nof_nsdu = cnt;
		}

		for (cnt = 0; cnt < CAN_BUS_CONTROLLER_CNT; cnt++) {
			cantp_channel_init(&cantp.connect[cnt].rx_chn);
			cantp_channel_init(&cantp.connect[cnt].tx_chn);
		}
	}
}

static void cantp_channel_init(cantp_channel_t *chn)
{
	chn->xfer_total_cnt = 0;
	chn->xfer_cnt = 0;
	chn->state = 0;
}

static int32_t cantp_nsdu_lookup_sduid(cantp_nsdu_t **nsdu, uint32_t sdu_id)
{
	int32_t existed = 0;
	uint32_t index, sdu_cnt = cantp.nof_nsdu;
	cantp_nsdu_t *nsdu_list = cantp.nsdu_list;
	if (NULL != nsdu_list) {
		for (index = 0; index < sdu_cnt; index++) {
			if (sdu_id == nsdu_list->sdu_id) {
				existed = 1;
				*nsdu = nsdu_list;
				break;
			} else {
				nsdu_list++;
			}
		}
	}
	return existed;
}

static int32_t cantp_nsdu_lookup_pduid(cantp_nsdu_t **nsdu, uint32_t pdu_id)
{
	int32_t existed = 0;
	uint32_t index, sdu_cnt = cantp.nof_nsdu;
	cantp_nsdu_t *nsdu_list = cantp.nsdu_list;
	if (NULL != nsdu_list) {
		for (index = 0; index < sdu_cnt; index++) {
			if (pdu_id == nsdu_list->rx_pdu_id) {
				existed = 1;
				*nsdu = nsdu_list;
				break;
			} else {
				/* Nothing to do! */
			}
			nsdu_list++;
		}
	}
	return existed;
}

int32_t cantp_transmit(uint32_t sdu_id, pdu_t *pdu)
{
	int32_t result = E_OK;

	uint32_t cpu_sts;
	//uint32_t frame_type;
	cantp_channel_t *tx_chn = NULL;
	cantp_nsdu_t *tx_nsdu = NULL;

	do {
		if ((NULL == pdu) || (0 == cantp_nsdu_lookup_sduid(&tx_nsdu, sdu_id))) {
			result = E_PAR;
			break;
		}

		/* lookup connected transmission channel. */
		if (tx_nsdu->connect_id >= CAN_BUS_CONTROLLER_CNT) {
			result = E_SYS;
			break;
		}
		tx_chn = &cantp.connect[tx_nsdu->connect_id].tx_chn;

		cpu_sts = arc_lock_save();
		if (CANTP_TX_IDLE != tx_chn->state) {
			result = E_DBUSY;
			arc_unlock_restore(cpu_sts);
			break;
		}
		tx_chn->state = CANTP_TX_WAIT_TRANSMIT;
		arc_unlock_restore(cpu_sts);

		/* clear message buffer. */
		tx_chn->buffer.length = 0;
		tx_chn->buffer.index = 0;

		/* init xfer info. */
		tx_chn->xfer_cnt = 0;
		tx_chn->msg_data = pdu->data;
		tx_chn->xfer_total_cnt = pdu->length;

		tx_chn->xfer_ctrl.tx_sn = 0;

		/* record current sdu and pdu. */
		tx_chn->sdu_id = sdu_id;
		tx_chn->pdu_id = tx_nsdu->tx_pdu_id;

		/* start a new transfer process, waiting to transmit.
		 * after Cs, if the state is still not into wait_tx_
		 * confirmation, timeout!
		 * */
		tx_chn->state_timeout = tx_nsdu->n_cs + 1;

		result = cantp_fill_frame(tx_nsdu, tx_chn);
		if (E_OK == result) {
			result = cantp_send_frame(tx_chn, tx_nsdu);
			if (E_OK != result) {
				tx_chn->state = CANTP_TX_IDLE;
			}
		}

	} while (0);

	return result;
}

#if 0
static uint32_t cantp_get_frame_type(cantp_channel_t *tx_chn, cantp_nsdu_t *tx_nsdu)
{
	uint32_t frame_type = 0;

	/* using extended address:
	 * means that extended address need an extra byte. */
	if (tx_nsdu->addr_xtd) {
		if (tx_chn->xfer_total_cnt <= 6) {
			frame_type = SINGLE_FRAME;
		} else {
			frame_type = FIRST_FRAME;
		}
	} else {
		if (tx_chn->xfer_total_cnt <= 7) {
			frame_type = SINGLE_FRAME;
		} else {
			frame_type = FIRST_FRAME;
		}
	}

	return frame_type;
}
#endif

static int32_t cantp_fill_frame(cantp_nsdu_t *tx_nsdu, cantp_channel_t *tx_chn)
{
	int32_t result = E_OK;

	uint32_t frame_type = 0;//cantp_get_frame_type(tx_chn, tx_nsdu);

	if (tx_nsdu->addr_xtd) {
		//tx_chn->buffer.data[tx_chn->buffer.index++] = tx_nsdu->n_ta;
		tx_chn->buffer.data[tx_chn->buffer.index++] = tx_nsdu->t_addr;
	}

	/* TODO: maybe need move to diagnostics header file.
	 * using extended address:
	 * means that extended address need an extra byte. */
	if (tx_nsdu->addr_xtd) {
		if (tx_chn->xfer_total_cnt <= 6) {
			frame_type = SINGLE_FRAME;
		} else {
			frame_type = FIRST_FRAME;
		}
	} else {
		if (tx_chn->xfer_total_cnt <= 7) {
			frame_type = SINGLE_FRAME;
		} else {
			frame_type = FIRST_FRAME;
		}
	}

	switch (frame_type) {
		case SINGLE_FRAME:
			tx_chn->buffer.data[tx_chn->buffer.index++] = \
				CANTP_PCI_SF | tx_chn->xfer_total_cnt;
			break;

		case FIRST_FRAME:
			tx_chn->buffer.data[tx_chn->buffer.index++] = \
				CANTP_PCI_FF | (tx_chn->xfer_total_cnt >> 8);
			tx_chn->buffer.data[tx_chn->buffer.index++] = \
				(tx_chn->xfer_total_cnt);

			/* after first frame, sender must receive FC to
			 * get the capability of receiver. */
			tx_chn->xfer_ctrl.next_fc_cnt = 1;
			tx_chn->xfer_ctrl.bs = 1;
			break;
		default:
			result = E_SYS;
	}

	return result;
}

static int32_t cantp_send_frame(cantp_channel_t *tx_chn, cantp_nsdu_t *tx_nsdu)
{
	int32_t result = E_OK;

	/* compute the data length. */
	uint32_t index, len = tx_nsdu->frame_length - tx_chn->buffer.index;
	if (len > tx_chn->xfer_total_cnt - tx_chn->xfer_cnt) {
		len = tx_chn->xfer_total_cnt - tx_chn->xfer_cnt;
	}

	if (tx_chn->msg_data) {
		pdu_t pdu;
		uint32_t remain_size = 0;

		/* copy data from upper layer. */
		pdu.data = &tx_chn->buffer.data[tx_chn->buffer.index];
		pdu.length = len;
		result = uds_sd_txdata_copy(tx_nsdu->sdu_id, &pdu, &remain_size);
		if (E_OK == result) {
			tx_chn->xfer_cnt += len;
			tx_chn->buffer.index += len;

			index = tx_chn->buffer.index;
			if (tx_nsdu->padding_active) {
				for (; index < tx_nsdu->frame_length; index++) {
					tx_chn->buffer.data[index] = 0x0;
				}
			}
			tx_chn->buffer.length = tx_nsdu->frame_length;

			pdu.data = tx_chn->buffer.data;
			pdu.length = tx_chn->buffer.length;
			result = canif_transmit(tx_chn->pdu_id, &pdu);
			if (E_OK == result) {
				tx_chn->state = CANTP_TX_WAIT_CONFIRMATION;
				tx_chn->state_timeout = tx_nsdu->n_as + 1;
			}
		}
	} else {
		result = E_SYS;
	}

	return result;
}

/* the transmission state machine. */
static void cantp_tx_handle(cantp_channel_t *tx_chn, cantp_nsdu_t *tx_nsdu)
{
	int32_t result = E_OK;

	switch (tx_chn->state) {
		case CANTP_TX_IDLE:
			/* continue to waiting, timeout control by upper layer. */
			break;

		case CANTP_TX_WAIT_TRANSMIT:
			result = cantp_send_frame(tx_chn, tx_nsdu);
			if (E_OK != result) {
				/* error: ignore the current transmission process. */
				tx_chn->state = CANTP_TX_IDLE;
				tx_chn->sdu_id = 0xFF;

				/* inform upper layer, transmitting failed! */
				uds_sd_confirmation(tx_chn->sdu_id, N_RESULT_ERROR);
			}
			break;

		case CANTP_TX_WAIT_CONFIRMATION:
			/* update state machine tick. */
			if (tx_chn->state_timeout > 0) {
				tx_chn->state_timeout -= 1;
			}

			/* wait lower confirmation timeout, report error confirmation. */
			if (0 == tx_chn->state_timeout) {
				tx_chn->state = CANTP_TX_IDLE;
				tx_chn->sdu_id = 0xFF;

				/* inform upper layer that waiting confirmation timeout. */
				uds_sd_confirmation(tx_chn->sdu_id, N_RESULT_TIMEOUT_A);
			}
			break;

		case CANTP_TX_WAIT_STMIN:
			if (tx_chn->state_timeout > 0) {
				tx_chn->state_timeout -= 1;
			}

			/* now can transmit next frame!
			 * switch state machine and set up timeout value(N_Cs).
			 * */
			if (0 == tx_chn->state_timeout) {
				tx_chn->state = CANTP_TX_WAIT_TRANSMIT;
				tx_chn->state_timeout = tx_nsdu->n_cs + 1;
			}
			break;

		case CANTP_TX_WAIT_FC:
			if (tx_chn->state_timeout > 0) {
				tx_chn->state_timeout -= 1;
			}

			/* wait flow frame timeout(N_Bs). */
			if (0 == tx_chn->state_timeout) {
				tx_chn->state = CANTP_TX_IDLE;
				tx_chn->sdu_id = 0xFF;

				/* inform upper layer, waiting FC timeout. */
				uds_sd_confirmation(tx_chn->sdu_id, N_RESULT_TIMEOUT_BS);
			}
			break;

		default:
			result = E_PAR;
	}

}

static inline int32_t cantp_tx_channel_lookup(cantp_channel_t **chn, uint32_t pdu_id)
{
	uint32_t index, existed = 0;

	cantp_channel_t *tx_chn = NULL;

	for (index = 0; index < CAN_BUS_CONTROLLER_CNT; index++) {
		tx_chn = &cantp.connect[index].tx_chn;
		if (pdu_id == tx_chn->pdu_id) {
			existed = 1;
			*chn = tx_chn;
			break;
		}
	}

	return existed;
}

void cantp_confirmation(uint32_t pdu_id)
{
	int32_t result = E_OK;

	//uint32_t index = 0;
	uint32_t flag = 0;
	uint32_t st_min_flow = 0;

	cantp_channel_t *tx_chn = NULL;
	cantp_nsdu_t *tx_nsdu = NULL;

	/* lookup transmission channel and sdu. */
	if (cantp_tx_channel_lookup(&tx_chn, pdu_id) && \
	    cantp_nsdu_lookup_sduid(&tx_nsdu, tx_chn->sdu_id)) {
		flag = 1;
	}

	if ((1 == flag) && (CANTP_TX_WAIT_CONFIRMATION == tx_chn->state)) {
		if (tx_chn->xfer_ctrl.next_fc_cnt > 0) {
			tx_chn->xfer_ctrl.next_fc_cnt -= 1;
		}

		if (tx_chn->xfer_total_cnt <= tx_chn->xfer_cnt) {
			/* finish sending entire message. */
			tx_chn->state = CANTP_TX_IDLE;
			tx_chn->sdu_id = 0xFF;
			uds_sd_confirmation(tx_chn->sdu_id, N_RESULT_OK);

		} else {
			/*
			 * the subsequence is finished(block size),
			 * now start waiting flow control frame.
			 * */
			if ((0 == tx_chn->xfer_ctrl.next_fc_cnt) &&\
			    (tx_chn->xfer_ctrl.bs)) {
				//tx_chn->xfer_ctrl.cur_expire = tx_nsdu->n_bs + 1;
				tx_chn->state_timeout = tx_nsdu->n_bs + 1;
				tx_chn->state = CANTP_TX_WAIT_FC;
			} else {
				/* check st_min. */
				st_min_flow = 1;
			}
		}
	}

	if (1 == st_min_flow) {
		if (0 == tx_chn->xfer_ctrl.st_min) {
			/* send next CF without waiting. */
			tx_chn->xfer_ctrl.tx_sn += 1;
			/*
			if (tx_chn->xfer_ctrl.tx_sn > 15) {
				tx_chn->xfer_ctrl.tx_sn = 0;
			}
			*/

			/* fill N_PCI. */
			tx_chn->buffer.index = 0;
			if (tx_nsdu->addr_xtd) {
				//tx_chn->buffer.data[tx_chn->buffer.index++] = tx_nsdu->n_ta;
				tx_chn->buffer.data[tx_chn->buffer.index++] = tx_nsdu->t_addr;
			}
			tx_chn->buffer.data[tx_chn->buffer.index++] = CANTP_PCI_CF | \
						(tx_chn->xfer_ctrl.tx_sn & 0xF);

			result = cantp_send_frame(tx_chn, tx_nsdu);
			if (E_OK != result) {
				/* inform upper layer that transmission error. */
				uds_sd_confirmation(tx_chn->sdu_id, N_RESULT_ERROR);

				tx_chn->state = CANTP_TX_IDLE;
				tx_chn->sdu_id = 0xFF;
			}

		} else {
			/* before sending next frame, waiting a while. */
			if (tx_chn->xfer_ctrl.st_min < 0x80) {
				tx_chn->state_timeout = tx_chn->xfer_ctrl.st_min + 1;
			} else if ((tx_chn->xfer_ctrl.st_min > 0xF0 ) && \
				   (tx_chn->xfer_ctrl.st_min < 0xFA)) {
				tx_chn->state_timeout = 1;
			} else {
				tx_chn->state_timeout = 0xFF;
			}

			tx_chn->state = CANTP_TX_WAIT_STMIN;
		}
	}
}

#if 0
static inline int32_t cantp_rx_nsdu_lookup(cantp_nsdu_t **rx_nsdu, uint32_t pdu_id)
{
	int32_t existed = 0;

	uint32_t nsdu_cnt = cantp_desc.nof_nsdu;
	cantp_nsdu_t *nsdu_list = cantp_desc.nsdu_list;

	if (NULL != nsdu_list) {
		uint32_t index = 0;
		for (; index < nsdu_cnt; index++) {
			/* pdu_id based on CAN ID. */
			if (pdu_id == nsdu_list->pdu_id) {
				existed = 1;
				*rx_nsdu = nsdu_list;
				break;
			} else {
				nsdu_list++;
			}
		}
	}

	return existed;
}
#endif

static uint32_t cantp_get_frametype(pdu_t *pdu)
{
	uint32_t frametype = 0xff;
	if ((NULL != pdu) && (NULL != pdu->data)) {
		frametype = (pdu->data[0] >> 4) & 0xF;
	}
	return frametype;
}

#if 0
static void trace_info(uint32_t value)
{
	static uint32_t idx = 0;
	raw_writel(0xa02000 + (idx++ << 2), value);
}
#endif

void cantp_indication(uint32_t pdu_id, pdu_t *pdu)
{
	int32_t result = E_OK;

	//uint32_t index = 0;
	//uint32_t flag = 0;
	uint32_t frame_type = 0;

	cantp_channel_t *rx_chn = NULL;
	cantp_channel_t *tx_chn = NULL;

	cantp_nsdu_t *rx_nsdu = NULL;

	if (cantp_nsdu_lookup_pduid(&rx_nsdu, pdu_id)) {
		/* get frame type. */
		frame_type = cantp_get_frametype(pdu);

		rx_chn = &cantp.connect[rx_nsdu->connect_id].rx_chn;
		switch (frame_type) {
			case SINGLE_FRAME:
				result = cantp_sf_handle(rx_chn, rx_nsdu, pdu);
				break;

			case FIRST_FRAME:
				result = cantp_ff_handle(rx_chn, rx_nsdu, pdu);
				break;

			case CONSECUTIVE_FRAME:
				result = cantp_cf_handle(rx_chn, rx_nsdu, pdu);
				break;

			case FLOW_CONTROL_FRAME:
				tx_chn = &cantp.connect[rx_nsdu->connect_id].tx_chn;
				result = cantp_fc_handle(tx_chn, rx_nsdu, pdu);
				break;

			default:
				/* Error: invalid frame! */
				result = E_SYS;
		}
	} else {
		result = E_SYS;
	}

	/* record error information. */
	if (E_OK != result) {
		;
	}
}

#if 0
static cantp_tx_nsdu_t *can_tp_get_tx_nsdu(uint32_t sdu_id)
{
	uint32_t index = 0;
	uint32_t flag = 0;
	cantp_tx_nsdu_t *tx_nsdu = n_sdu.tx_nsdu_list;
	for (; index < n_sdu.nof_tx_nsdu; index++) {
		if (adu_id == tx_nsdu->sdu_id) {
			flag = 1;
			break;
		} else {
			tx_nsdu++;
		}
	}

	if (0 == flag) {
		tx_nsdu = NULL;
	}
	return tx_nsdu;
}
#endif

static inline uint32_t cantp_get_sf_datalen(uint32_t addr_xtd, pdu_t *pdu)
{
	uint32_t len = 0;
	if (addr_xtd) {
		len = pdu->data[1] & 0xF;
	} else {
		len = pdu->data[0] & 0xF;
	}
	return len;
}
static void cantp_abort_rx(cantp_channel_t *rx_chn, cantp_nsdu_t *n_nsdu)
{
	/*
	switch (rx_chn->state) {
		case CANTP_RX_IDLE:
		case CANTP_RX_SF_WAIT_BUF:
		case CANTP_RX_CF_WAIT_BUF:
			break;
		case CANTP_RX_SF_ALLOC_BUF:
		case CANTP_RX_FF_ALLOC_BUF:
		case CANTP_RX_WAIT_CF:
		default:
			result = E_SYS;
	}
	*/
	cantp_channel_init(rx_chn);
}
static int32_t cantp_sf_handle(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, pdu_t *pdu)
{
	int32_t result = E_OK;

	uint32_t data_len = 0;
	uint32_t data_pos = 0;


	if ((NULL == rx_chn) || (NULL == rx_nsdu) || (NULL == pdu)) {
		result = E_PAR;
	} else {
		/* gain data length from frame. */
		data_len = cantp_get_sf_datalen(rx_nsdu->addr_xtd, pdu);
		if (rx_nsdu->addr_xtd) {
			uint32_t addr_xtd = pdu->data[0];
			//if ((addr_xtd != rx_nsdu->n_ta) ||
			if ((addr_xtd != rx_nsdu->t_addr) || \
			    (data_len > 6) || (0 == data_len)) {
				result = E_SYS;
			}
		} else {
			if ((0 == data_len) || (data_len > 7)) {
				result = E_SYS;
			}
		}
	}

	if (E_OK == result) {
		uint32_t remain_size = 0;

		/* 
		 * if channel is not idle, abort the current xfer process.
		 * and start a new receiving process.
		 * */
		cantp_abort_rx(rx_chn, rx_nsdu);

		if (rx_nsdu->addr_xtd) {
			data_pos = 2;
		} else {
			data_pos = 1;
		}

		rx_chn->xfer_total_cnt = data_len;

		rx_chn->sdu_id = rx_nsdu->sdu_id;

		rx_chn->state_timeout = rx_nsdu->n_br + 1;
		rx_chn->state = CANTP_RX_SF_ALLOC_BUF;

		/* alloc and copy data upto upper module. */
		result = uds_sd_malloc(rx_nsdu->sdu_id, data_len, &remain_size);
		if (E_OK == result) {
			pdu_t upper_pdu;

			upper_pdu.data = &pdu->data[data_pos];
			upper_pdu.length = data_len;

			/* begin to copy data. */
			//rx_chn->state = CANTP_RX_SF_COPY_DATA;
			result = uds_sd_rxdata_copy(rx_nsdu->sdu_id, &upper_pdu, &remain_size);
			if (E_OK == result) {
				/* inform upper layer, data is ready. */
				uds_sd_indication(rx_chn->sdu_id, N_RESULT_OK);

				rx_chn->state = CANTP_RX_IDLE;
				rx_chn->pdu_id = 0xFF;
			}
		}

		/* buffer busy, store data at local buffer! */
		if (E_DBUSY == result) {
			uint32_t index = 0;
			for (; index < data_len; index++) {
				rx_chn->buffer.data[index] = pdu->data[data_pos++];
			}
			rx_chn->buffer.length = data_len;

			rx_chn->state = CANTP_RX_SF_WAIT_BUF;
		} else {
			/* error: record! */
			rx_chn->state = CANTP_RX_IDLE;
			rx_chn->pdu_id = 0xFF;
		}
	}

	return result;
}

static uint32_t cantp_get_ff_datalen(uint32_t addr_xtd, pdu_t *pdu)
{
	uint32_t pos = 0;
	if (addr_xtd) {
		pos = 1;
	}
	return (((pdu->data[pos] & 0xF) << 8) | pdu->data[pos + 1]);
}
static int32_t cantp_ff_handle(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, pdu_t *pdu)
{
	int32_t result = E_OK;

	uint32_t data_len = 0;
	uint32_t data_pos = 0;

	if ((NULL == rx_chn) || (NULL == rx_nsdu) || (NULL == pdu)) {
		result = E_PAR;
	} else {
		data_len = cantp_get_ff_datalen(rx_nsdu->addr_xtd, pdu);
		if (rx_nsdu->addr_xtd) {
			uint32_t xtd_addr = pdu->data[0];
			//if ((xtd_addr != rx_nsdu->n_ta) || (data_len < 6)) {
			if ((xtd_addr != rx_nsdu->t_addr) || (data_len < 6)) {
				result = E_SYS;
			}
			data_pos = 3;
		} else {
			if (data_len < 7) {
				result = E_SYS;
			}
			data_pos = 2;
		}
	}

	if (E_OK == result) {
		uint32_t remain_size = 0;

		/* if channel is not idle, abort the current xfer process. */
		cantp_abort_rx(rx_chn, rx_nsdu);
		//rx_chn->xfer_ctrl.rx_sn = 0;
		rx_chn->xfer_ctrl.rx_sn = 1;//rx_sn should be 1, because the consecutive frame SN will be 1 after the flow control frame.

		rx_chn->xfer_total_cnt = data_len;
		rx_chn->sdu_id = rx_nsdu->sdu_id;
		rx_chn->state_timeout = rx_nsdu->n_br + 1;
		rx_chn->state = CANTP_RX_FF_ALLOC_BUF;

		/* alloc and copy data upto upper module. */
		result = uds_sd_malloc(rx_nsdu->sdu_id, data_len, &remain_size);
		if (E_OK == result) {
			pdu_t upper_pdu;

			upper_pdu.data = &pdu->data[data_pos];
			upper_pdu.length = pdu->length - data_pos;

			/* begin to copy data. */
			//rx_chn->state = CANTP_RX_SF_COPY_DATA;
			result = uds_sd_rxdata_copy(rx_nsdu->sdu_id, &upper_pdu, &remain_size);
			if (E_OK == result) {
				rx_chn->xfer_cnt += upper_pdu.length;
				rx_chn->state_timeout = rx_nsdu->n_cr + 1;
				rx_chn->state = CANTP_RX_WAIT_CF;

				/* send flow control frame:
				 * tell sendor the capability. */
				cantp_send_fc(rx_chn, rx_nsdu, CANTP_FS_CTS);
			}
		}

		if (E_DBUSY == result) {
			uint32_t index = 0;
			data_pos += 1;
			for (; index < pdu->length - data_pos; index++) {
				rx_chn->buffer.data[index] = pdu->data[data_pos++];
			}
			rx_chn->buffer.length = data_len;

			rx_chn->state_timeout = rx_nsdu->n_br + 1;
			rx_chn->state = CANTP_RX_CF_WAIT_BUF;

			/* send flow control frame: let sendor wait. */
			cantp_send_fc(rx_chn, rx_nsdu, CANTP_FS_WAIT);
		} else {
			if (E_OK != result) {
				rx_chn->state = CANTP_RX_IDLE;
				rx_chn->pdu_id = 0xFF;
			}
		}
	}

	return result;
}

static int32_t cantp_cf_handle(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, pdu_t *pdu)
{
	int32_t result = E_OK;

	uint32_t sn = 0;
	uint32_t data_pos = 0;

	if ((NULL == rx_chn) || (NULL == rx_nsdu) || (NULL == pdu)) {
		result = E_PAR;
	} else {
		if (rx_nsdu->addr_xtd) {
			uint32_t xtd_addr = pdu->data[0];
			sn = pdu->data[1] & 0xF;
			//if (xtd_addr != rx_nsdu->n_ta) {
			if (xtd_addr != rx_nsdu->t_addr) {
				result = E_SYS;
			}
			data_pos = 2;
		} else {
			sn = pdu->data[0] & 0xF;
			data_pos = 1;
		}

		if ((E_OK == result) && (sn != rx_chn->xfer_ctrl.rx_sn)) {
			uds_sd_indication(rx_chn->sdu_id, N_RESULT_WRONG_SN);

			rx_chn->pdu_id = 0xFF;
			rx_chn->state = CANTP_RX_IDLE;
			result = E_SYS;
		}

		if ((E_OK == result) && (rx_chn->state != CANTP_RX_WAIT_CF)) {
			result = E_SYS;
		}
	}

	if (E_OK == result) {
		uint32_t remain_size = 0;
		//uint32_t cur_valid_cnt = 0;
		uint32_t data_cnt = rx_nsdu->frame_length - data_pos;
		uint32_t remain_cnt = rx_chn->xfer_total_cnt - rx_chn->xfer_cnt;

		pdu_t upper_pdu;

		remain_cnt = rx_chn->xfer_total_cnt - rx_chn->xfer_cnt;
		upper_pdu.data = &pdu->data[data_pos];
		if (remain_cnt > pdu->length - data_pos) {
			upper_pdu.length = pdu->length - data_pos;
		} else {
			upper_pdu.length = remain_cnt;
		}

		if (remain_cnt < data_cnt) {
			//cur_valid_cnt = remain_cnt;
		} else {
			//cur_valid_cnt = data_cnt;
		}

		/* begin to copy data. */
		//rx_chn->state = CANTP_RX_SF_COPY_DATA;
		result = uds_sd_rxdata_copy(rx_nsdu->sdu_id, &upper_pdu, &remain_size);
		if (E_OK == result) {
			rx_chn->xfer_cnt += upper_pdu.length;
			remain_cnt = rx_chn->xfer_total_cnt - rx_chn->xfer_cnt;
			if (remain_cnt > 0) {
				rx_chn->xfer_ctrl.rx_sn++;
				if (rx_chn->xfer_ctrl.rx_sn >= 0x10) {
					rx_chn->xfer_ctrl.rx_sn = 0;
				}

				if (rx_chn->xfer_ctrl.next_fc_cnt > 0) {
					rx_chn->xfer_ctrl.next_fc_cnt--;
				}
				if ((0 == rx_chn->xfer_ctrl.next_fc_cnt) && \
				    (rx_chn->xfer_ctrl.bs > 0)) {
					/* send flow control frame: start a new process. */
					cantp_send_fc(rx_chn, rx_nsdu, CANTP_FS_CTS);
				} else {
					//rx_chn->cur_expire = rx_nsdu->n_cr + 1;
					rx_chn->state_timeout = rx_nsdu->n_cr + 1;
				}
			} else {
				rx_chn->state = CANTP_RX_IDLE;
				rx_chn->pdu_id = 0xFF;

				uds_sd_indication(rx_chn->sdu_id, N_RESULT_OK);
			}

		//} else if (E_BUSY == result) {
			/* maybe while rx_chn->xfer_ctrl.next_fc_cnt = 1,
			 * this frame can be stored on local buffer.
			 * */
		} else {
			uds_sd_indication(rx_chn->sdu_id, N_RESULT_ERROR);

			rx_chn->state = CANTP_RX_IDLE;
			rx_chn->pdu_id = 0xFF;
		}
	}

	return result;
}


static int32_t cantp_fc_handle(cantp_channel_t *tx_chn, cantp_nsdu_t *tx_nsdu, pdu_t *pdu)
{
	int32_t result = E_OK;

	uint32_t index = 0;

	if ((NULL == tx_chn) || (NULL == tx_nsdu) || (NULL == pdu)) {
		result = E_PAR;
	} else {
		/* extend addressing. */
		if (tx_nsdu->addr_xtd) {
			uint32_t xtd_addr = pdu->data[index++];
			//if (xtd_addr != tx_nsdu->n_sa) {
			if (xtd_addr != tx_nsdu->s_addr) {
				result = E_SYS;
			}
		}
	}

	if ((E_OK == result) && (CANTP_TX_WAIT_FC == tx_chn->state)) {
		/* parse flow status from framw data. */
		uint32_t fc_status = pdu->data[index++] & 0xF;

		switch (fc_status) {
			case CANTP_FS_CTS:
				tx_chn->xfer_ctrl.bs = pdu->data[index++];
				tx_chn->xfer_ctrl.next_fc_cnt = tx_chn->xfer_ctrl.bs;
				tx_chn->xfer_ctrl.st_min = pdu->data[index++];

				tx_chn->state_timeout = tx_nsdu->n_cs + 1;
				tx_chn->state = CANTP_TX_WAIT_TRANSMIT;
				break;

			case CANTP_FS_WAIT:
				/* continue waiting a new flow control frame. */
				tx_chn->state_timeout = tx_nsdu->n_bs + 1;
				break;

			case CANTP_FS_OVERFLOW:
				/* inform upper layer receiver over flow. */
				uds_sd_confirmation(tx_chn->sdu_id, N_RESULT_BUFFER_OVFLW);

				tx_chn->state = CANTP_TX_IDLE;
				tx_chn->sdu_id = 0xFF;
				break;

			default:
				/* inform upper layer that received a invalid flow status. */
				uds_sd_confirmation(tx_chn->sdu_id, N_RESULT_INVALID_FS);

				tx_chn->state = CANTP_TX_IDLE;
				tx_chn->sdu_id = 0xFF;

				result = E_SYS;
		}
	}

	return result;
}

static void cantp_rx_handle(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu)
{
	int32_t result = E_OK;

	uint32_t data_len = 0, remain_size = 0;

	switch (rx_chn->state) {
		case CANTP_RX_IDLE:
			/* nothing to do! */
			break;

		case CANTP_RX_SF_ALLOC_BUF:
		case CANTP_RX_FF_ALLOC_BUF:
			/* temp state! */
			break;

		case CANTP_RX_SF_WAIT_BUF:
			if (rx_chn->state_timeout > 0) {
				rx_chn->state_timeout -= 1;
			}

			/* allocate buffer and copy data to upper module from local buffer. */
			result = uds_sd_malloc(rx_nsdu->sdu_id, data_len, &remain_size);
			if (E_OK == result) {
				pdu_t upper_pdu;

				upper_pdu.data = rx_chn->buffer.data;
				upper_pdu.length = rx_chn->buffer.length;

				/* begin to copy data. */
				//rx_chn->state = CANTP_RX_SF_COPY_DATA;
				result = uds_sd_rxdata_copy(rx_nsdu->sdu_id, &upper_pdu, &remain_size);
				if (E_OK == result) {
					/* inform upper layer, data is ready. */
					uds_sd_confirmation(rx_chn->sdu_id, N_RESULT_OK);

					rx_chn->state = CANTP_RX_IDLE;
					rx_chn->pdu_id = 0xFF;
				}
			}

			if (E_DBUSY == result) {
				if (0 == rx_chn->state_timeout) {
					rx_chn->state = CANTP_RX_IDLE;
					rx_chn->pdu_id = 0xFF;
				} else {
					/* continue to wait upper module. */
				}
			} else {
				/* Error:  */
				rx_chn->state = CANTP_RX_IDLE;
				rx_chn->pdu_id = 0xFF;
			}
			break;

		case CANTP_RX_CF_WAIT_BUF:
			if (rx_chn->state_timeout > 0) {
				rx_chn->state_timeout -= 1;
			}

			/* allocate buffer and copy data to upper module from local buffer. */
			result = uds_sd_malloc(rx_nsdu->sdu_id, data_len, &remain_size);
			if (E_OK == result) {
				pdu_t upper_pdu;

				upper_pdu.data = rx_chn->buffer.data;
				upper_pdu.length = rx_chn->buffer.length;

				/* begin to copy data. */
				//rx_chn->state = CANTP_RX_SF_COPY_DATA;
				result = uds_sd_rxdata_copy(rx_nsdu->sdu_id, &upper_pdu, &remain_size);
				if (E_OK == result) {
					if (rx_chn->xfer_total_cnt - rx_chn->xfer_cnt > 0) {
						/* the sender has been flow controlled.
						 * so firstly unassert it. */
						/* send flow control frame: let sendor recovered to transmit. */
						cantp_send_fc(rx_chn, rx_nsdu, CANTP_FS_CTS);

						rx_chn->state_timeout = rx_nsdu->n_cr + 1;
						rx_chn->state = CANTP_TX_WAIT_FC;
					} else {
						rx_chn->pdu_id = 0xFF;
						rx_chn->state = CANTP_RX_IDLE;

						/* inform upper layer that data is ready. */
						uds_sd_confirmation(rx_chn->sdu_id, N_RESULT_OK);
					}
				}
			}

			if (E_DBUSY == result) {
				uint32_t wftmax_0 = 0;
				if (0 == rx_chn->state_timeout) {
					if (rx_chn->xfer_ctrl.wft_max_cnt < rx_nsdu->wft_max) {
						/* send flow control frame: let sender wait. */
						cantp_send_fc(rx_chn, rx_nsdu, CANTP_FS_WAIT);
						rx_chn->state_timeout = rx_nsdu->n_br + 1;
					} else {
						wftmax_0 = 1;
					}
				} else {
					if (rx_chn->xfer_ctrl.wft_max_cnt >= rx_nsdu->wft_max) {
						wftmax_0 = 1;
					}
				}

				if (1 == wftmax_0) {
					rx_chn->xfer_ctrl.wft_max_cnt = 0;
					rx_chn->pdu_id = 0xFF;
					rx_chn->state = CANTP_RX_IDLE;

					uds_sd_confirmation(rx_chn->sdu_id, N_RESULT_ERROR);
				}
			} else {
				rx_chn->pdu_id = 0xFF;
				rx_chn->state = CANTP_RX_IDLE;

				uds_sd_confirmation(rx_chn->sdu_id, N_RESULT_ERROR);
			}
			break;

		case CANTP_RX_WAIT_CF:
			if (rx_chn->state_timeout > 0) {
				rx_chn->state_timeout -= 1;
			}
			if (0 == rx_chn->state_timeout) {
				rx_chn->pdu_id = 0xFF;
				rx_chn->state = CANTP_RX_IDLE;

				uds_sd_confirmation(rx_chn->sdu_id, N_RESULT_ERROR);
			}
			break;
		default:
			break;
	}
}

static void cantp_send_fc(cantp_channel_t *rx_chn, cantp_nsdu_t *rx_nsdu, uint32_t status)
{
	int32_t result = E_OK;

	uint32_t index = 0;
	uint8_t fc_data[8];
	pdu_t pdu = {
		.data = &fc_data[0],
	};

	if (rx_nsdu->addr_xtd) {
		//fc_data[index++] = rx_nsdu->n_sa;
		fc_data[index++] = rx_nsdu->s_addr;
	}

	switch (status) {
		case CANTP_FS_CTS:
			rx_chn->xfer_ctrl.upper_buf_size = uds_sd_xfer_capability();
			/* send bs and st_min. */
			fc_data[index++] = CANTP_PCI_FC | CANTP_FS_CTS;
			if (rx_nsdu->addr_xtd) {
				fc_data[index++] = rx_chn->xfer_ctrl.upper_buf_size / 6;
			} else {
				fc_data[index++] = rx_chn->xfer_ctrl.upper_buf_size / 7;
			}
			rx_chn->xfer_ctrl.bs = fc_data[index - 1];
			rx_chn->xfer_ctrl.next_fc_cnt = rx_chn->xfer_ctrl.bs;
			fc_data[index++] = rx_nsdu->st_min;
			pdu.length = index;
			rx_chn->xfer_ctrl.wft_max_cnt = 0;
			break;

		case CANTP_FS_WAIT:
			fc_data[index++] = CANTP_PCI_FC | CANTP_FS_WAIT;
			pdu.length = index + 2;
			rx_chn->xfer_ctrl.wft_max_cnt++;
			break;

		case CANTP_FS_OVERFLOW:
			fc_data[index++] = CANTP_PCI_FC | CANTP_FS_OVERFLOW;
			pdu.length = index + 2;
			break;

		default:
			result = E_SYS;
	}

	if (E_OK == result) {
		if (rx_nsdu->padding_active) {
			for (index = pdu.length; index < rx_nsdu->frame_length; index++) {
				pdu.data[index] = 0x55;
			}
			pdu.length = rx_nsdu->frame_length;
		}
		result = canif_transmit(rx_nsdu->tx_pdu_id, &pdu);
		if (E_OK == result) {
			/* TODO: state --> wait confirmation? */
		} else {
			/* TODO: what to do? */
		}
	}
}

/* session state machine: while tick timer timeout, it'll be called. */
void cantp_state_main(void)
{
	cantp_connect_t *connect = NULL;
	cantp_channel_t *txchn = NULL;
	cantp_channel_t *rxchn = NULL;
	cantp_nsdu_t *rx_nsdu = NULL;
	cantp_nsdu_t *tx_nsdu = NULL;

	uint32_t con_cnt = CAN_BUS_CONTROLLER_CNT;

	while (con_cnt--) {
		connect = &cantp.connect[con_cnt];
		rxchn = &connect->rx_chn;
		txchn = &connect->tx_chn;

		if (CANTP_RX_IDLE != rxchn->state) {
			if (cantp_nsdu_lookup_sduid(&rx_nsdu, rxchn->sdu_id)) {
				cantp_rx_handle(rxchn, rx_nsdu);
			}
		}

		if (CANTP_TX_IDLE != txchn->state) {
			if (cantp_nsdu_lookup_sduid(&tx_nsdu, txchn->sdu_id)) {
				cantp_tx_handle(rxchn, tx_nsdu);
			}
		}
	}
}
