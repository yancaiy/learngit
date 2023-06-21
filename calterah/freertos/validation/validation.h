#ifndef VALIDATION_H
#define VALIDATION_H


/* @rt_flag: */
#define VSCASE_RT_STOP		(1 << 0)

/*
 * Return:
 *     0     : Case finished with success.
 *     1     : Case ongoing.
 *     -1    : Case finished with fail.
 *     Other : Case internal error.
 */
typedef int32_t (*case_init)(void *self, void *params, uint32_t len);
typedef int32_t (*case_deinit)(void *self);
typedef int32_t (*case_entry)(void *self, void *params, uint32_t len);
typedef int32_t (*case_exit)(void *self);

typedef struct {
	uint16_t mod_id;
	uint16_t case_id;

	uint32_t rt_flag;

	case_entry  entry;
	case_exit   exit;
	case_init   init;
	case_deinit deinit;
} validation_case_t;

typedef struct auto_test_parameters {

	uint32_t id;
	uint32_t baudrate;
	uint32_t type;
	uint32_t len;
	uint32_t self_define;

} auto_test_param_t;

#define VALIDATION_CASE_DEFINE(MODULE, mid, cid, f_entry, f_exit) \
static __attribute__ ((section(".vcase_table"))) volatile validation_case_t validation_case_##MODULE##_##cid = { \
	.mod_id = mid, \
	.case_id = cid, \
	.entry = f_entry, \
	.exit = f_exit \
}

#define VALIDATION_SYS_CASE_DEFINE(mid, cid, f_init, f_deinit) \
__attribute__ ((section(".vscase_table"))) volatile validation_case_t validation_case_os_##mid##_##cid = { \
	.mod_id = mid, \
	.case_id = cid, \
	.init = f_init, \
	.deinit = f_deinit, \
}

void validation_entry(void);
void validation_os_entry(void);
int  validation_os_entry_console(uint8_t oper, uint32_t mid, uint32_t cid, uint8_t* param, uint32_t param_len);

/* while received "stop" command, stop_task will set stop flag.
 * user can call this function to get the flag. */
int32_t vscase_over(uint16_t mid, uint16_t cid);

uint32_t get_byte_from_param(void** params);
uint32_t get_word_from_param(void** params);
void param_analy(void* params, uint32_t len, auto_test_param_t *config);

#endif
