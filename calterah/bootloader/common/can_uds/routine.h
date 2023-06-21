#ifndef ROUTINE_H
#define ROUTINE_H

#define RID_NOR_FLASH_ERASE		(0x0)
#define RID_NOR_FLASH_PROG		(0x1)
#define ROUTINE_ID_MAX			(0x4)

typedef enum {
	RC_ISO_SAE_RESERVED = 0,
	RC_START_ROUTINE,
	RC_STOP_ROUTINE,
	RC_REQUEST_ROUTINE_RESULT,
	RC_TYPE_MAX
} routine_control_type_t;
#define RC_TYPE_NUM_MAX			(RC_TYPE_MAX)

typedef uint8_t (*routine_callback)(uint8_t *params, uint32_t params_len);

typedef struct {
	routine_control_type_t func_id;
	uint16_t record_len;
	routine_callback func;
} routine_func_t;

typedef struct {
	uint16_t id;
	routine_func_t *func_table;
} uds_routine_t;


uds_routine_t *uds_routines_get(uint32_t *cnt);

uint8_t routine_nor_flash_erase(uint8_t *params, uint32_t params_len);

#endif
