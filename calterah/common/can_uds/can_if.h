#ifndef CAN_IF_H
#define CAN_IF_H

#include "can_hal.h"

typedef enum can_id_type {
	CAN_ID_STD = 0,
	CAN_ID_XTD,
	CAN_ID_TYPE_INVALID
} can_idtype_t;

typedef enum can_bus_type {
	CAN_BUS_BASIC = 0,
	CAN_BUS_FULL,
	CAN_BUS_TYPE_INVALID
} can_bus_type_t;

typedef struct can_if_tx_pdu_config {
	uint32_t pdu_id;

	uint32_t can_id;
	can_idtype_t id_type;

	uint8_t dlc;

	/*
	uint32_t buf_id_index;
	uint32_t buf_cnt;
	*/
	//int32_t (*tx_confirmation)(uint32_t txpdu_id);
	uint8_t tx_confirmation_id;

	can_bus_type_t bus_type;
	uint32_t hw_chn_id;
} canif_txpdu_cfg_t;

typedef struct {
	uint32_t min_can_id;
	uint32_t max_can_id;
} canif_id_range_t;

typedef struct can_if_rx_pdu_config {
	uint32_t pdu_id;

	canif_id_range_t id_range;
	can_idtype_t id_type;
	uint8_t dlc;

	//int32_t (*rx_indication)(uint32_t rxpdu_id, pdu_t *pdu);
	uint8_t rx_indication_id;

	can_bus_type_t bus_type;
	uint32_t hw_chn_id;
} canif_rxpdu_cfg_t;

#if 0
typedef struct {
	uint32_t pdu_id;
	uint32_t can_id;
	uint32_t length;
	uint8_t *data;
} can_lpdu_t;
#endif

typedef struct {
	uint8_t *data;
	uint32_t length;
} pdu_t;

#if 0
typedef void (*canif_txconfirmation)(uint8_t pdu_id);
typedef void (*canif_rxindication)(uint8_t pdu_id, pdu_t *pdu_info);
#else
typedef void (*canif_txconfirmation)(uint32_t pdu_id);
typedef void (*canif_rxindication)(uint32_t pdu_id, pdu_t *pdu_info);
#endif

void canif_init(void);
int32_t canif_transmit(uint32_t pdu_id, pdu_t *pdu);
#ifdef CAN_TX_COMPLETE_MONITOR
void canif_tx_confirmation_conver(uint32_t msg_id, uint32_t isFailed);
#else
void canif_tx_confirmation_conver(uint32_t msg_id);
#endif
void canif_tx_confirmation(uint32_t pdu_id);
void canif_rx_indication_conver(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len);
void canif_rx_indication(uint32_t chn_id, uint32_t can_id, uint32_t dlc, uint8_t *data);
//int32_t canif_set_controller_mode(uint32_t ctrl_id, can_controller_state_t state);
//int32_t canif_get_controller_mode(uint32_t ctrl_id, can_controller_state_t *state);
void canif_callback_register(uint32_t txpdu_id, canif_txconfirmation txfunc, uint32_t rxpdu_id, canif_rxindication rxfunc);

#endif
