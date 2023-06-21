#ifndef INTERPRETER_H
#define INTERPRETER_H

typedef enum {
	VSCASE_START = 1,
	VSCASE_STOP
} vscase_cmd_t;

int32_t interpreter_case_parameter(uint16_t *mid, uint16_t *cid, uint8_t **data, uint32_t *len);
int32_t interpreter_vscase_stop(void);

#endif
