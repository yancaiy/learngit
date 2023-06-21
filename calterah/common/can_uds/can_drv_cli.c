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


#if 0
static volatile uint32_t send_done = 0;
static uint32_t txdata[0x1000];

static uint32_t candrv_transmit_done(void);
static BaseType_t candrv_command_handle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	int32_t result = E_OK;

	int32_t xfer_done = 0;
	static uint32_t first_xfer = 0;
	can_pdu_t pduinfo;

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
		xfer_done = candrv_transmit_done();
	}

	pduinfo.pdu_id = 0;
	pduinfo.length = 8;
	pduinfo.can_id = 0x12;
	pduinfo.sdu = (uint8_t *)txdata;

	result = can_write(0, &pduinfo);
	if (E_OK != result) {
		EMBARC_PRINTF("can write failed%d\r\n", result);
	}

	return pdFALSE;
}

static const CLI_Command_Definition_t candrv_send_command = {
	"can_send",
	"\rcan_send"
	"\r\n\tcan_send [length(word)]\r\n\r\n",
	candrv_command_handle,
	-1
};

void candrv_server_register(void)
{
	FreeRTOS_CLIRegisterCommand(&candrv_send_command);
}

static void can_send_done(uint32_t pdu_id)
{
	send_done = 1;
}

static uint32_t candrv_transmit_done(void)
{
	while (0 == send_done);
	send_done = 0;
	return 1;
}

static void can_rx_done(uint32_t rx_chn, uint32_t can_id, uint32_t dlc, uint8_t *data_buffer)
{
	EMBARC_PRINTF("chn%d, id:0x%x, dlc: 0x%x.\r\n", rx_chn, can_id, dlc);
	EMBARC_PRINTF("0x%x 0x%x 0x%x 0x%x ", *data_buffer++, *data_buffer++, *data_buffer++, *data_buffer++);
	EMBARC_PRINTF("0x%x 0x%x 0x%x 0x%x\r\n", *data_buffer++, *data_buffer++, *data_buffer++, *data_buffer++);
}

void can_bus_init(void)
{
	can_config_t *config = (can_config_t *)can_config_get();
	if (NULL != config) {
		can_init(config);
		can_callback_register(0, can_rx_done, can_send_done);
		candrv_server_register();
		can_enable_controller_interrupt(0);
	}
}
#endif
