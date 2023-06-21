#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_error.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "board.h"
#include "dw_ssi.h"
#include "xip_hal.h"
#include "device.h"
#include "instruction.h"
#include "cfi.h"
#include "sfdp.h"
#include "flash.h"
#include "spi_hal.h"

#ifdef OS_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"

#define FLASH_TEST_COMMAND		(1)

#if FLASH_TEST_COMMAND
static uint32_t flash_test_cmd_buffer[1024];

static BaseType_t flash_erase_command_handle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1, len2;
	uint32_t addr, length;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	addr = (uint32_t)strtol(param1, NULL, 0);
	length = (uint32_t)strtol(param2, NULL, 0);

	EMBARC_PRINTF("addr: 0x%x, length: 0x%x\r\n", addr, length);
	flash_memory_erase(addr, length);

	return pdFALSE;
}
static const CLI_Command_Definition_t flash_erase_command =
{
	"flash_erase",
	"\rflash_erase:"
	"\r\n\tflash_erase [address] [length(byte)]\r\n\r\n",
	flash_erase_command_handle,
	-1
};

static BaseType_t flash_memory_write_command_handle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1, len2, len3;
	uint32_t addr, data, length;

	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	const char *param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);
	addr = (uint32_t)strtol(param1, NULL, 0);
	data = (uint32_t)strtol(param2, NULL, 0);
	length = (uint32_t)strtol(param3, NULL, 0);
	if (length > 1024)
	{
		EMBARC_PRINTF("length: %d is too large, max write length is 1024 words\r\n", length);
		length = 1024;
	}

	EMBARC_PRINTF("addr: 0x%x, data: 0x%x length: 0x%x\r\n", addr, data, length);

	for (uint32_t i = 0; i < length; i++) {
		flash_test_cmd_buffer[i] = (i << 16) | (data++);
	}

	flash_memory_writew(addr, flash_test_cmd_buffer, length);

	return pdFALSE;
}

static const CLI_Command_Definition_t flash_memory_write_command =
{
	"flash_memory_write",
	"\rflash_memory_write:"
	"\r\n\tflash_memory_write [address] [length(words)] most 1024\r\n\r\n",
	flash_memory_write_command_handle,
	-1
};

static BaseType_t flash_memory_read_command_handle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1, len2;
	uint32_t addr, length;

	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	addr = (uint32_t)strtol(param1, NULL, 0);
	length = (uint32_t)strtol(param2, NULL, 0);
	if (length > 1024)
	{
		EMBARC_PRINTF("length: %d is too large, max write length is 1024 words\r\n", length);
		length = 1024;
	}

	flash_memory_readw(addr, flash_test_cmd_buffer, length);

	chip_hw_mdelay(100);

	for (uint32_t i = 0; i < length; i++) {
		if ((i == 0) || (i % 4 == 0)) {
			EMBARC_PRINTF("%08x", (addr + (i * 4)));
		}

		if ((i + 1) % 4 == 0) {
			EMBARC_PRINTF("\t %08x \r\n", flash_test_cmd_buffer[i]);
		} else {
			EMBARC_PRINTF("\t %08x", flash_test_cmd_buffer[i]);
		}
	}

	return pdFALSE;
}

static const CLI_Command_Definition_t flash_memory_read_command =
{
	"flash_memory_read",
	"\rflash_memory_read:"
	"\r\n\tflash_memory_read [address] [length(words)]\r\n\r\n",
	flash_memory_read_command_handle,
	-1
};

#endif

void flash_cli_register(void)
{
#if FLASH_TEST_COMMAND
	FreeRTOS_CLIRegisterCommand(&flash_erase_command);
	FreeRTOS_CLIRegisterCommand(&flash_memory_write_command);
	FreeRTOS_CLIRegisterCommand(&flash_memory_read_command);
#endif
}
#endif
