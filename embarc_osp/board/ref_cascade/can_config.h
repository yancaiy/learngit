#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H



void *canif_get_txpdu_cfg(uint32_t *cnt);
void *canif_get_rxpdu_cfg(uint32_t *cnt);
void *cantp_get_nsdu(uint32_t *cnt);
void *can_config_get(void);
#endif
