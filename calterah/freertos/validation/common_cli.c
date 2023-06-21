#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"
#include "command.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#include "flash_header.h"
#include "common_cli.h"
#include "radio_ctrl.h"

#include <stdlib.h>
#include <string.h>
#include "validation.h"

#ifdef CPU_STAT
#include "task.h"
static uint8_t cpu_usage_info[128];
#endif

/* uart_ota_cli */
static BaseType_t uart_ota_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t ota_command =
{
        "uart_ota",
        "\r\nuart_ota:\r\n reprogram firmare through uart interface\r\n\r\n",
        uart_ota_handler,
        0
};

static BaseType_t uart_ota_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
        fmcw_radio_reboot_cause_set(ECU_REBOOT_UART_OTA);

        chip_hw_mdelay(1);

        fmcw_radio_reset();

        return 0;
}

void uart_ota_init(void)
{
        FreeRTOS_CLIRegisterCommand(&ota_command);
}


/* firmware_version_cli */
static bool common_cli_registered = false;
static BaseType_t fw_ver_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t fw_ver_cmd = {
        "fw_ver",
        "fw_ver \n\r"
        "\tGet firmware software version. \n\r"
        "\tUsage: fw_ver \n\r",
        fw_ver_handler,
        -1
};

static BaseType_t fw_ver_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
#if (defined SYSTEM_3_STAGES_BOOT) && (defined __FIRMWARE__)
        image_header_t *flash_header = (image_header_t *)flash_header_get();
#else
        flash_header_t *flash_header = (flash_header_t *)flash_header_get();
#endif

#if FLASH_XIP_EN == 1
        snprintf(pcWriteBuffer, xWriteBufferLen, \
                "On XIP, Version: %d.%d.%d \n\rBuild Data: %s\n\rBuild Time: %s \n\rSystem Name:%s\n\rBuild commit ID: %s\n\r", \
                flash_header->sw_version.major_ver_id, \
                flash_header->sw_version.minor_ver_id, \
                flash_header->sw_version.stage_ver_id, \
                flash_header->sw_version.date, \
                flash_header->sw_version.time, \
                flash_header->sw_version.info, \
                BUILD_COMMIT_ID
        );
#else
        snprintf(pcWriteBuffer, xWriteBufferLen, \
                "On SRAM, Version: %d.%d.%d \n\rBuild Data: %s\n\rBuild Time: %s \n\rSystem Name:%s\n\rBuild commit ID: %s\n\r", \
                flash_header->sw_version.major_ver_id, \
                flash_header->sw_version.minor_ver_id, \
                flash_header->sw_version.stage_ver_id, \
                flash_header->sw_version.date, \
                flash_header->sw_version.time, \
                flash_header->sw_version.info, \
                BUILD_COMMIT_ID
        );
#endif

        return pdFALSE;
}

static uint32_t mem_read(const uint32_t addr)
{
        volatile uint32_t *ptr = (uint32_t *)addr;
        return *ptr;
}

static BaseType_t mem_dump_command_handle(char *pcWriteBuffer, \
		size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1, len2;
	uint32_t addr, length;
	uint32_t buffer;

	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	addr = (uint32_t)strtol(param1, NULL, 0);
	length = (uint32_t)strtol(param2, NULL, 0);

	for (uint32_t i = 0; i < length; i++) {
		if ((i == 0) || (i % 4 == 0)) {
			EMBARC_PRINTF("%08x", (addr + (i * 4)));
		}

		buffer = mem_read(addr + i*4);
		if ((i+1) % 4 == 0) {
			EMBARC_PRINTF("\t %08x \r\n", buffer);
		} else {
			EMBARC_PRINTF("\t %08x", buffer);
		}
	}

	return pdFALSE;
}

static const CLI_Command_Definition_t mem_dump_command =
{
	"mem_dump",
	"\rmem_dump:"
	"\r\n\tmem_dump [address] [length(word)]\r\n\r\n",
	mem_dump_command_handle,
	-1
};


static BaseType_t chip_reset_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t chip_reset_command = {
	"chip_reset",
	"chip_reset \n\r"
	"\tChip reset. \n\r"
	"\tUsage: chip_reset \n\r",
	chip_reset_command_handler,
	-1
};


static BaseType_t chip_reset_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
        fmcw_radio_reset();

	return pdFALSE;
}

#ifdef CPU_STAT
static BaseType_t cpu_usage_stat_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
        memset(cpu_usage_info, 0, 128);
        vTaskGetRunTimeStats((char *)&cpu_usage_info);
        EMBARC_PRINTF("Task\t\tTime(us)\tUsage(%%)\r\n");
        EMBARC_PRINTF("%s", cpu_usage_info);
        return pdFALSE;
}

static const CLI_Command_Definition_t cpu_usage_stat_command = {
        "cpu_usage",
        "cpu_usage statistics\n\r",
        cpu_usage_stat_command_handler,
        0
};
#endif

/* validation cli */
static BaseType_t validation_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static const CLI_Command_Definition_t validation_test_cmd = {
        "tm",
        "test module \n\r"
        "\tValidation test command \n\r"
        "\tUsage: tm <start/stop> <module id> <case id> [param]\n\r",
        validation_handler,
        -1
};

static BaseType_t validation_handler( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    int ret;
    uint8_t oper;
    uint32_t mid, cid;
    BaseType_t len1, len2, len3, len4;
    const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
    const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
    const char *param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);
    const char *param4 = FreeRTOS_CLIGetParameter(pcCommandString, 4, &len4);

    /* Check parameters */
    if ((param1 == NULL) || (param2 == NULL) || (param3 == NULL)) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error, you should input 3 parameters at least!\r\n");
        return pdFALSE;
    }

    /* Get MID and CID, from hex string */
    mid = (uint32_t)strtol(param2, NULL, 0);
    cid = (uint32_t)strtol(param3, NULL, 0);

    /* Parse Operation */
    if (strncmp(param1, "start", sizeof("start") - 1) == 0) {
        oper = 0;
    } else if (strncmp(param1, "stop", sizeof("stop") - 1) == 0) {
        oper = 1;
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error, first parameter should be: start/stop!\r\n");
        return pdFALSE;
    }

    EMBARC_PRINTF("[validation_handler] %s, mid= %d, cid= %d!\r\n", oper?"stop":"start", mid, cid);

    // Run the validation case
    ret = validation_os_entry_console(oper, mid, cid, (uint8_t*)param4, len4);
    if (ret == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Validation Success : %s, mid %d, cid %d!\r\n", oper?"stop":"start", mid, cid);
    } else if (ret == 1) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Validation Ongoing : %s, mid %d, cid %d!\r\n", oper?"stop":"start", mid, cid);
    } else if (ret == -1) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Validation Fail    : %s, mid %d, cid %d!\r\n", oper?"stop":"start", mid, cid);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Validation Error   : %s, return value: %d!\r\n", oper?"stop":"start", ret);
    }
    return pdFALSE;
}

void common_cmd_init(void)
{
        if (common_cli_registered)
                return;

        FreeRTOS_CLIRegisterCommand(&fw_ver_cmd);
        FreeRTOS_CLIRegisterCommand(&mem_dump_command);
        FreeRTOS_CLIRegisterCommand(&chip_reset_command);
#ifdef CPU_STAT
        FreeRTOS_CLIRegisterCommand(&cpu_usage_stat_command);
#endif

        FreeRTOS_CLIRegisterCommand(&validation_test_cmd);
        common_cli_registered = true;

        return;
}
