#include <string.h>
#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "arc_exception.h"

//#include "can_bus_config.h"
#include "dev_can.h"
#include "can.h"
#include "can_hal.h"
#include "can_if.h"

#include "can_config.h"

/* move to can_config.h */
#define CANIF_TXPDU_MAX		(2)
#define CANIF_RXPDU_MAX		(2)

typedef struct {
	/* 0->idle, 1->pending(waiting for deal). */
	uint8_t state;
	uint8_t pdu_id;
	pdu_t pdu;
	uint8_t data[64];
} canif_pdu_buffer_t;

typedef struct {
	uint32_t inited;

	uint32_t nof_rx_pdu;
	uint32_t nof_tx_pdu;
	canif_txpdu_cfg_t *tx_pdu;
	canif_rxpdu_cfg_t *rx_pdu;

	//canif_id_range_t rx_id_range[SYS_CAN_CONTROLLER_CNT];
	/* can transceiver config... */
} canif_global_t;

/*********************** local variables *********************************/
static canif_txconfirmation tx_confirmation_list[CANIF_TXPDU_MAX];
static canif_rxindication rx_indication_list[CANIF_RXPDU_MAX];

static canif_pdu_buffer_t local_txpdu_buffer[CANIF_TXPDU_MAX];

static canif_global_t canif_desc;


/************************ local functions description ************************/
static int32_t canif_txpdu_lookup(canif_txpdu_cfg_t **pdu, uint32_t pdu_id);
static int32_t canif_rxpdu_lookup(canif_rxpdu_cfg_t **pdu, uint32_t chn_id, uint32_t can_id);
static int32_t canif_txpdu_lookup_by_can_id(canif_txpdu_cfg_t **pdu, uint32_t hw_chn_id, uint32_t can_id);

void canif_init(void)
{
	// int32_t result = E_OK;
	uint32_t count = 0;

	canif_desc.tx_pdu = (canif_txpdu_cfg_t *)canif_get_txpdu_cfg(&count);
	if (canif_desc.tx_pdu) {
		canif_desc.nof_tx_pdu = count;
	}

	count = 0;
	canif_desc.rx_pdu = (canif_rxpdu_cfg_t *)canif_get_rxpdu_cfg(&count);
	if (canif_desc.rx_pdu) {
		canif_desc.nof_rx_pdu = count;
	}

	//can_callback_register(dev_can->id, canif_rx_indication, canif_tx_confirmation);
	//can_enable_controller_interrupt(dev_can->id);

        can_init(0, CAN_BAUDRATE_500KBPS, 0);
        can_indication_register(0, canif_rx_indication_conver);
        can_confirmation_register(0, canif_tx_confirmation_conver);

        /* Enable the RX interrupt */
        can_interrupt_enable(0, 0, 1);
        /* Enable the error interrupt */
        can_interrupt_enable(0, 3, 1);
        /* Enable the TX interrupt */
        can_interrupt_enable(0, 1, 1);

	canif_desc.inited = 1;
}

static inline uint32_t byte2word(uint8_t *buf, uint32_t *length)
{
        uint32_t raw_data = 0;
        if (*length) {
                uint32_t i, min_cnt = 4;
                if (*length < min_cnt) {
                        min_cnt = *length;
                        *length = 0;
                } else {
                        *length -= 4;
                }
                for (i = 0; i < min_cnt; i++) {
                        raw_data |= buf[i] << (i << 3);
                }
        }
        return raw_data;
}

static inline void word2byte(uint8_t *buf, uint32_t *length, uint32_t data)
{
        if ((NULL != buf) && (NULL != length) && (*length)) {
                uint32_t i, min_cnt = 4;
                if (*length < min_cnt) {
                        min_cnt = *length;
                        *length = 0;
                } else {
                        *length -= 4;
                }
                for (i = 0; i < min_cnt; i++) {
                        buf[i] = (data >> (i << 3)) & 0xFF;
                }
        }
}

int32_t canif_transmit(uint32_t pdu_id, pdu_t *pdu)
{
    int32_t result = E_OK;
    uint32_t transmit_data[2] = {0};
    canif_txpdu_cfg_t *tx_pdu = NULL;
    can_config_t can_param;

    if (canif_txpdu_lookup(&tx_pdu, pdu_id)) {
        /* upper layer to deal with return value.
         * if lower layer is busy, upper layer can
         * buffer this frame on its local buffer.
         * */
        uint32_t len = pdu->length;
        transmit_data[0] = byte2word(pdu->data, &len);
        if(len > 0)
                    transmit_data[1] = byte2word(pdu->data+4, &len);
        can_get_config(tx_pdu->hw_chn_id , (void*)(&can_param));
        if (can_param.data_tx_mode != DEV_XFER_INTERRUPT){
            can_param.data_tx_mode = DEV_XFER_INTERRUPT;
            can_set_config(tx_pdu->hw_chn_id, (void*)(&can_param));
        }
        result = can_send_data_isr(tx_pdu->hw_chn_id, tx_pdu->can_id, transmit_data, tx_pdu->dlc);
        if (E_OK != result) {
            uint32_t idx;
            canif_pdu_buffer_t *pdu_buffer = NULL;
            uint32_t cpu_sts = arc_lock_save();
            for (idx = 0; idx < CANIF_TXPDU_MAX; idx++) {
                if (0 == local_txpdu_buffer[idx].state) {
                    pdu_buffer = &local_txpdu_buffer[idx];
                    pdu_buffer->state = 1;
                    break;
                }
            }
            arc_unlock_restore(cpu_sts);
            if (pdu_buffer) {
                pdu_buffer->pdu_id = pdu_id;
                pdu_buffer->pdu.data = pdu->data;
                pdu_buffer->pdu.length = pdu->length;
                memcpy(pdu_buffer->data, pdu->data, pdu->length);
                result = E_OK;
            }
        }

    } else {
        result = E_PAR;
    }

    return result;
}

//can_if tx confirmation conversion to adapt can_hal interface
#ifdef CAN_TX_COMPLETE_MONITOR
void canif_tx_confirmation_conver(uint32_t msg_id, uint32_t isFailed)
#else
void canif_tx_confirmation_conver(uint32_t msg_id)
#endif
{
        //int32_t result = E_OK;

        canif_txpdu_cfg_t *tx_pdu = NULL;
        // msg_id -> pdu_id
        if(1 == canif_txpdu_lookup_by_can_id(&tx_pdu, 0, msg_id))
        {
                canif_tx_confirmation(tx_pdu->pdu_id);
        } else {
                //result = E_SYS;
                /* TODO: record an error! */
        }
}

void canif_tx_confirmation(uint32_t pdu_id)
{
	int32_t result = E_OK;

	canif_txpdu_cfg_t *tx_pdu = NULL;

	if (canif_txpdu_lookup(&tx_pdu, pdu_id)) {
		/* if achieved local buffer for transmission, and if
		 * there is a pdu buffered in local buffer. send it firstly.
		 * */
		uint32_t idx;
		canif_pdu_buffer_t *pdu_buffer = NULL;
		//uint32_t cpu_sts = arc_lock_save();
		for (idx = 0; idx < CANIF_TXPDU_MAX; idx++) {
			if (1 == local_txpdu_buffer[idx].state) {
				canif_txpdu_cfg_t *pending_txpdu = NULL;
				result = canif_txpdu_lookup(&pending_txpdu, local_txpdu_buffer[idx].pdu_id);
				if (result) {
					if(tx_pdu->hw_chn_id == pending_txpdu->hw_chn_id) {
						pdu_buffer = &local_txpdu_buffer[idx];
						break;
					}
				}
			}
		}
		//arc_unlock_restore(cpu_sts);
		if (pdu_buffer) {
			pdu_buffer->pdu.data = pdu_buffer->data;
			result = canif_transmit(pdu_buffer->pdu_id, &pdu_buffer->pdu);
			if (E_OK == result) {
				pdu_buffer->state = 0;
			}
		}

#if 0
		/* reserved option:
		 * each pdu can have own confirmation callback.
		 * */
		if (tx_pdu->confirmation_callback) {
			tx_pdu->confirmation_callback(pdu_id);
		}
#endif

		/* confirmation to transport part. */
		if (CANIF_TXPDU_MAX > tx_pdu->tx_confirmation_id) {
			if (tx_confirmation_list[tx_pdu->tx_confirmation_id]) {
				tx_confirmation_list[tx_pdu->tx_confirmation_id](pdu_id);
			}
		}

	} else {
		result = E_SYS;
		/* TODO: record an error! */
	}

}

//can_if rx indication conversion to adapt can_hal interface
void canif_rx_indication_conver(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
        uint8_t  data_in_bytes[8];
        uint32_t data_len;

        data_len = len;
        word2byte(&data_in_bytes[0], &data_len, data[0]);
        if(data_len > 0)
                word2byte(&data_in_bytes[4], &data_len, data[1]);
        canif_rx_indication(0, msg_id, len, data_in_bytes);
}

void canif_rx_indication(uint32_t chn_id, uint32_t can_id, uint32_t dlc, uint8_t *data)
{
	//int32_t result = E_OK;
	canif_rxpdu_cfg_t *rx_pdu = NULL;

	if (canif_rxpdu_lookup(&rx_pdu, chn_id, can_id)) {
		pdu_t pdu_info;
		pdu_info.data = data;
		pdu_info.length = dlc;

#if 0
		/* reserved option:
		 * each pdu can have own indication callback.
		 * */
		if (rx_pdu->rx_indication) {
			result = rx_pdu->rx_indication(rx_pdu->pdu_id, &rx_pdu_info);
			if (E_OK != result) {
				/* TODO: record an error! */
			}
		}
#endif

		if (CANIF_RXPDU_MAX > rx_pdu->rx_indication_id) {
			if (rx_indication_list[rx_pdu->rx_indication_id]) {
				rx_indication_list[rx_pdu->rx_indication_id](rx_pdu->pdu_id, &pdu_info);
			}
		}

	} else {
		/* TODO: record an error! */
	}
}

/*int32_t canif_set_controller_mode(uint32_t ctrl_id, can_controller_state_t state)
{
	return can_set_controller_mode(ctrl_id, state);
}

int32_t canif_get_controller_mode(uint32_t ctrl_id, can_controller_state_t *state)
{
	return can_get_controller_mode(ctrl_id, state);
}*/

void canif_callback_register(uint32_t txpdu_id, canif_txconfirmation txfunc, uint32_t rxpdu_id, canif_rxindication rxfunc)
{
	uint32_t cpu_sts = arc_lock_save();
	if ((txpdu_id < CANIF_TXPDU_MAX) && (NULL != txfunc)) {
		tx_confirmation_list[txpdu_id] = txfunc;
	}
	if ((txpdu_id < CANIF_RXPDU_MAX) && (NULL != rxfunc)) {
		rx_indication_list[rxpdu_id] = rxfunc;
	}
	arc_unlock_restore(cpu_sts);
}

/************************ local functions **************************/
static int32_t canif_txpdu_lookup(canif_txpdu_cfg_t **pdu, uint32_t pdu_id)
{
	int32_t existed = 0;

	uint32_t pdu_cnt = canif_desc.nof_tx_pdu;
	canif_txpdu_cfg_t *pdu_list = canif_desc.tx_pdu;

	while (pdu_cnt--) {
		if (pdu_list->pdu_id == pdu_id) {
			existed = 1;
			*pdu = pdu_list;
			break;
		} else {
			pdu_list++;
		}
	}
	return existed;
}

static int32_t canif_rxpdu_lookup(canif_rxpdu_cfg_t **pdu, uint32_t chn_id, uint32_t can_id)
{
	int32_t existed = 0;

	uint32_t pdu_cnt = canif_desc.nof_rx_pdu;
	canif_rxpdu_cfg_t *pdu_list = canif_desc.rx_pdu;

	while (pdu_cnt--) {
		if ((pdu_list->hw_chn_id == chn_id) && \
		    (can_id >= pdu_list->id_range.min_can_id) &&\
		    (can_id <= pdu_list->id_range.max_can_id)) {
			existed = 1;
			*pdu = pdu_list;
			break;
		} else {
			pdu_list++;
		}
	}
	return existed;
}

static int32_t canif_txpdu_lookup_by_can_id(canif_txpdu_cfg_t **pdu, uint32_t hw_chn_id, uint32_t can_id)
{
        int32_t existed = 0;

        uint32_t pdu_cnt = canif_desc.nof_tx_pdu;
        canif_txpdu_cfg_t *pdu_list = canif_desc.tx_pdu;

        while (pdu_cnt--) {
                if (pdu_list->can_id == can_id && pdu_list->hw_chn_id == hw_chn_id) {
                        existed = 1;
                        *pdu = pdu_list;
                        break;
                } else {
                        pdu_list++;
                }
        }
        return existed;
}
