#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_error.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "board.h"

#ifdef OS_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#endif
#include "can_hal.h"
#include "can_if.h"

#if 0
static volatile uint32_t send_done = 0;
static uint32_t txdata[0x1000];

static uint32_t canif_transmit_done(void);
static BaseType_t canif_command_handle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	int32_t result = E_OK;

	int32_t xfer_done = 0;
	static uint32_t first_xfer = 0;
	pdu_t pduinfo;

        BaseType_t len1;
	uint32_t idx, length;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	length = (uint32_t)strtol(param1, NULL, 0);

	EMBARC_PRINTF("txdata: 0x%x, length: 0x%x\r\n", (uint32_t)txdata, length);
	for (idx = 0; idx < length; idx++) {
		txdata[idx] = 0x12340000 + idx;
	}

	if (0 == first_xfer) {
		first_xfer = 1;
	} else {
		xfer_done = canif_transmit_done();
	}

	pduinfo.length = 8;
	pduinfo.data = (uint8_t *)txdata;

	result = canif_transmit(0, &pduinfo);
	if (E_OK != result) {
		EMBARC_PRINTF("can write failed%d\r\n", result);
	}

	return pdFALSE;
}

static const CLI_Command_Definition_t canif_send_command = {
	"can_send",
	"\rcan_send"
	"\r\n\tcan_send [length(word)]\r\n\r\n",
	canif_command_handle,
	-1
};

void canif_server_register(void)
{
	FreeRTOS_CLIRegisterCommand(&canif_send_command);
}

static void canif_send_done(uint32_t pdu_id)
{
	send_done = 1;
}

static uint32_t canif_transmit_done(void)
{
	while (0 == send_done);
	send_done = 0;
	return 1;
}

static void canif_rx_done(uint32_t pdu_id, pdu_t *pduinfo)
{
	uint8_t *data_buffer = pduinfo->data;
	EMBARC_PRINTF("0x%x 0x%x 0x%x 0x%x ", *data_buffer++, *data_buffer++, *data_buffer++, *data_buffer++);
	EMBARC_PRINTF("0x%x 0x%x 0x%x 0x%x\r\n", *data_buffer++, *data_buffer++, *data_buffer++, *data_buffer);
}

void can_bus_init(void)
{
	can_config_t *config = (can_config_t *)can_config_get();
	if (NULL != config) {
		can_init(config);
		canif_init();
		canif_callback_register(0, canif_send_done, 1, canif_rx_done);
		canif_server_register();
	}
}
#endif
