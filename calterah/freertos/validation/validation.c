#include "embARC.h"
#include "embARC_debug.h"
#include "validation.h"
#include "interpreter.h"
#include "uart_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stdlib.h"

extern uint32_t _f_vcase;
extern uint32_t _f_svcase;
extern uint32_t _f_os_vcase;
extern uint32_t _f_os_svcase;


#if BB_SHARED_MEM_EN == 1
/* uint: byte. */
#define VCASE_PARAMS_LEN_MAX        (0x1000)
static TaskHandle_t test_task_handle = NULL;
static uint8_t parameters[VCASE_PARAMS_LEN_MAX];
static void vscase_stop_task(void *params);
#endif

void validation_entry(void)
{
#if BB_SHARED_MEM_EN == 1
        int32_t result = E_OK;

        uint8_t *payload_ptr = parameters;
        uint16_t mid = 0, cid = 0;
        uint32_t payload_len = VCASE_PARAMS_LEN_MAX;

        uint32_t nof_vcase = 0;
        uint32_t vcase_table_base = (uint32_t)&_f_vcase;
        uint32_t vcase_table_end = (uint32_t)&_f_svcase;

        validation_case_t *vcase_ptr = NULL;

        EMBARC_PRINTF("[validation_entry] enter!\r\n");

        if (vcase_table_end > vcase_table_base) {
                nof_vcase = vcase_table_end - vcase_table_base;
                nof_vcase = nof_vcase / sizeof(validation_case_t);
                vcase_ptr = (validation_case_t *)vcase_table_base;
        }

        while (nof_vcase) {
                uint32_t idx = 0;
                vcase_ptr = (validation_case_t *)vcase_table_base;
                uart_flush(CHIP_CONSOLE_UART_ID);
                payload_ptr = parameters;
                payload_len = VCASE_PARAMS_LEN_MAX;
                result = interpreter_case_parameter(&mid, &cid, &payload_ptr, &payload_len);
                if (VSCASE_STOP == result) {
                        break;
                }
                if (0 != result) {
                        continue;
                }

                for (idx = 0; idx < nof_vcase; idx++) {
                        if ((mid == vcase_ptr->mod_id) && (cid == vcase_ptr->case_id)) {
                                break;
                        }
                        vcase_ptr++;
                }
                if (idx >= nof_vcase) {
                        EMBARC_PRINTF("[test] no module%d case%d.\r\n", mid, cid);
                        continue;
                }

                if ((NULL != vcase_ptr) && (NULL != vcase_ptr->entry)) {
                        result = vcase_ptr->entry(vcase_ptr, payload_ptr, payload_len);

                        EMBARC_PRINTF("[test] %d %d %d.\r\n", mid, cid, result);

                        if (vcase_ptr->exit) {
                                result = vcase_ptr->exit(vcase_ptr);
                        }
                } else {
                        EMBARC_PRINTF("[test] %d %d is not registered or achieved.\r\n", mid, cid);
                }
        }
        EMBARC_PRINTF("[validation_entry] exit!\r\n");
#endif
}

void validation_os_entry(void)
{
#if BB_SHARED_MEM_EN == 1
        int32_t result = E_OK;

        uint8_t *payload_ptr = parameters;
        uint16_t mid = 0, cid = 0;
        uint32_t payload_len = VCASE_PARAMS_LEN_MAX;

        uint32_t nof_vscase = 0;
        uint32_t vscase_table_base = (uint32_t)&_f_os_vcase;
        uint32_t vscase_table_end = (uint32_t)&_f_os_svcase;

        EMBARC_PRINTF("[validation_os_entry] enter!\r\n");

        validation_case_t *vscase_ptr = NULL;
        if (vscase_table_end > vscase_table_base) {
                nof_vscase = vscase_table_end - vscase_table_base;
                nof_vscase = nof_vscase / sizeof(validation_case_t);
                vscase_ptr = (validation_case_t *)vscase_table_base;
        }

        while (nof_vscase) {
                uint32_t idx = 0;
                vscase_ptr = (validation_case_t *)vscase_table_base;
                uart_flush(CHIP_CONSOLE_UART_ID);
                payload_ptr = parameters;
                payload_len = VCASE_PARAMS_LEN_MAX;
                result = interpreter_case_parameter(&mid, &cid, &payload_ptr, &payload_len);
                if (VSCASE_START == result) {
                        break;
                }
                if (0 != result) {
                        continue;
                }

                for (idx = 0; idx < nof_vscase; idx++) {
                        if ((mid == vscase_ptr->mod_id) && (cid == vscase_ptr->case_id)) {
                                break;
                        }
                        vscase_ptr++;
                }
                if (idx >= nof_vscase) {
                        EMBARC_PRINTF("[test] no module%d case%d.\r\n", mid, cid);
                        continue;
                }

                if ((NULL != vscase_ptr) && (NULL != vscase_ptr->init)) {
                        result = vscase_ptr->init(vscase_ptr, payload_ptr, payload_len);
                        if (E_OK != result) {
                                EMBARC_PRINTF("validation OS - Case: %d init failed.\r\n", vscase_ptr->case_id);
                        } else {
                                EMBARC_PRINTF("validation OS - Case: %d init success.\r\n", vscase_ptr->case_id);
                        }
                }
        }

        if (pdPASS != xTaskCreate(vscase_stop_task, \
                        "validation_task", 256, (void *)1, (configMAX_PRIORITIES - 1), &test_task_handle)) {
                EMBARC_PRINTF("create validation task error\r\n");
        }

    EMBARC_PRINTF("[validation_os_entry] exit!\r\n");
#endif
}

/*
 * Input:
 *     oper      : 0 - Start run case, 1 - Stop case.
 *     mid       : module ID.
 *     cid       : case ID.
 *     param     : Hex Str parameters, which will be bypass to case.
 *     Param_len : Length of param.
 * Return:
 *     0     : Case finished with success.
 *     1     : Case ongoing.
 *     -1    : Case finished with fail.
 *     -2    : Case not found.
 *     Other : Case internal error.
 */
int validation_os_entry_console(uint8_t oper, uint32_t mid, uint32_t cid, uint8_t* param, uint32_t param_len)
{
    int32_t result = E_OK;
    uint32_t idx = 0;
    uint32_t nof_vscase = 0;
    uint32_t vscase_table_base = (uint32_t)&_f_os_vcase;
    uint32_t vscase_table_end = (uint32_t)&_f_os_svcase;

    validation_case_t *vscase_ptr = NULL;
    if (vscase_table_end > vscase_table_base) {
        nof_vscase = vscase_table_end - vscase_table_base;
        nof_vscase = nof_vscase / sizeof(validation_case_t);
        vscase_ptr = (validation_case_t *)vscase_table_base;
    }

    EMBARC_PRINTF("[Validation] - nof_vscase:   %d.\r\n", nof_vscase);

    /* Search in case list. */
    vscase_ptr = (validation_case_t *)vscase_table_base;
    for (idx = 0; idx < nof_vscase; idx++) {
        
        if ((mid == vscase_ptr->mod_id) && (cid == vscase_ptr->case_id)) {
            EMBARC_PRINTF("[Validation] - vscase_ptr->mod_id: %d vscase_ptr->case_id: %d.\r\n", vscase_ptr->mod_id, vscase_ptr->case_id);
            break;
        }
        vscase_ptr++;
    }
    if (idx >= nof_vscase) {
        /* Case not found, so We return. */
        EMBARC_PRINTF("[Validation] - Not Found, module: %d case: %d.\r\n", mid, cid);
        return -2;
    }

    /* Case found. */

    if ((NULL == vscase_ptr) || (NULL == vscase_ptr->init)) {
        EMBARC_PRINTF("[Validation] - Invalid vscase_ptr or vscase_ptr->init.\r\n");
        return -3;
    }

    /* Start Operation. */
    if (oper == 0) {
        /* Operation: Start Case */
        EMBARC_PRINTF("[Validation] - Run, Module: %d, Case: %d.\r\n", vscase_ptr->mod_id, vscase_ptr->case_id);

        result = vscase_ptr->init(vscase_ptr, param, param_len);
        EMBARC_PRINTF("[Validation] - Run, Module: %d, Case: %d return: %d.\r\n", vscase_ptr->mod_id, vscase_ptr->case_id, result);
    } else if (oper == 1) {
        /* Operation: Stop Case */
        EMBARC_PRINTF("[Validation] - Stop, Module: %d, Case: %d.\r\n", vscase_ptr->mod_id, vscase_ptr->case_id);
        if (NULL != vscase_ptr->deinit) {
            result = vscase_ptr->deinit(vscase_ptr);
            EMBARC_PRINTF("[Validation] - Stop, Module: %d, Case: %d return: %d.\r\n", vscase_ptr->mod_id, vscase_ptr->case_id, result);
        } else {
            EMBARC_PRINTF("[Validation] - Warning, deinit function not defined.\r\n");
        }
    } else {
        /* Operation: Undefined */
        EMBARC_PRINTF("[Validation] - Error, invalid operation: %d.\r\n", oper);
        return -4;
    }

    return result;
}

int32_t vscase_over(uint16_t mid, uint16_t cid)
{
#if BB_SHARED_MEM_EN == 1
        int32_t result = E_OK;

        uint32_t idx, nof_vscase = 0;

        uint32_t vscase_table_base = (uint32_t)&_f_os_vcase;
        uint32_t vscase_table_end = (uint32_t)&_f_os_svcase;

        validation_case_t *vscase_ptr = NULL;
        if (vscase_table_end > vscase_table_base) {
                nof_vscase = vscase_table_end - vscase_table_base;
                nof_vscase = nof_vscase / sizeof(validation_case_t);
                vscase_ptr = (validation_case_t *)vscase_table_base;
        }

        for (idx = 0; idx < nof_vscase; idx++) {
                if ((mid == vscase_ptr->mod_id) && (cid == vscase_ptr->case_id)) {
                        if (vscase_ptr->rt_flag & VSCASE_RT_STOP) {
                                vscase_ptr->rt_flag &= ~VSCASE_RT_STOP;
                                result = 1;
                                break;
                        }
                }
                vscase_ptr++;
        }

        return result;
#else
        return 0;
#endif

}

static void vscase_stop_task(void *params)
{
#if BB_SHARED_MEM_EN == 1

        uint32_t idx, nof_vscase = 0;

        uint32_t vscase_table_base = (uint32_t)&_f_os_vcase;
        uint32_t vscase_table_end = (uint32_t)&_f_os_svcase;

        validation_case_t *vscase_ptr = NULL;
        if (vscase_table_end > vscase_table_base) {
                nof_vscase = vscase_table_end - vscase_table_base;
                nof_vscase = nof_vscase / sizeof(validation_case_t);
                vscase_ptr = (validation_case_t *)vscase_table_base;
        }

        while (1) {
                if (interpreter_vscase_stop()) {
                        EMBARC_PRINTF("uart stop enter!\r\n");

                        for (idx = 0; idx < nof_vscase; idx++) {
                                vscase_ptr->rt_flag = VSCASE_RT_STOP;
                                vscase_ptr++;
                        }
                } else {
                         vTaskDelay(1);
                }
        }
#endif
}

static void *tm_memcpy(void *dst, const void *src, uint32_t len)
{
	unsigned char *pSRC, *pDST;
	pSRC = (unsigned char *)src;
	pDST = (unsigned char *)dst;
	while (len--) {
		*pDST++ = *pSRC++;
	}
	return (void *)dst;
}

uint32_t get_byte_from_param(void** params)
{
        char word[8 + 1];

        tm_memcpy(word, *params, 2);

        word[2] = 0;

        *params = *params + 2;

        return (uint32_t)strtol(word, NULL, 16);
}

uint32_t get_word_from_param(void** params)
{
	char word[8 + 1];

	tm_memcpy(word, *params, 8);

	word[8] = 0;

	*params = *params + 8;

	return (uint32_t)strtol(word, NULL, 16);
}

void param_analy(void* params, uint32_t len, auto_test_param_t *config) {

	if (len < 5 * 8) {
		EMBARC_PRINTF("param len should >= 40");
		return;
	}

	config->id = get_word_from_param(&params);
	config->baudrate = get_word_from_param(&params);
	config->type = get_word_from_param(&params);
	config->len = get_word_from_param(&params);
	config->self_define = get_word_from_param(&params);

}