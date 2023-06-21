#include "embARC_assert.h"
#include <string.h>
#include <portmacro.h>
#include <FreeRTOS_CLI.h>
#include <projdefs.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "track_cli.h"
#include "can_signal_interface.h"
#include "can_hal.h"

static bool track_cli_registered = false;

volatile enum OBJECT_OUTPUT new_format;

static BaseType_t track_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
        const char *param1 = NULL, *param2 = NULL, *param3 = NULL;
        BaseType_t len1 = 0, len2 = 0, len3 = 0;
        can_config_t can_param;

        /* param1 : new_format -> config new data output format; NULL -> get current data output format   */
        /* param2 : 0 -> config new format as UART STRING;       1 -> config new format as UART HEX;      */
        /*        : 2 -> config new format as CAN/CANFD POLLING; 3 -> config new format as CAN/CANFD INT; */
        /* param3 : only use when data output format is CAN/CANFD        */
        /*        : bk -> only output BK data; ak -> only output AK data */
        param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
        param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
        param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);

        if (param1 != NULL) {
            if (strncmp(param1, "new_format", len1) == 0) {
                if (param2 != NULL) {

                    new_format = (int8_t)strtol(param2, NULL, 0);

                    switch (new_format) {
                    case UART_STRING:
                        snprintf(pcWriteBuffer, xWriteBufferLen, "UART STRING\n\r");
                        break;
                    case UART_HEX:
                        snprintf(pcWriteBuffer, xWriteBufferLen, "UART HEX\n\r");
                        break;
                    case CAN_POLLING:
                        can_get_config(CAN_0_ID, &can_param);
                        if(can_param.can_mode == CAN_FD_MODE){
                            snprintf(pcWriteBuffer, xWriteBufferLen, "CAN FD\n\r");
                        }else{
                            snprintf(pcWriteBuffer, xWriteBufferLen, "CAN\n\r");
                        }
                        if (DEV_XFER_POLLING != can_param.data_tx_mode){
                            can_param.data_tx_mode = DEV_XFER_POLLING;
                            can_set_config(CAN_0_ID,&can_param);
                        }
                        break;
                    case CAN_INT:
                        can_get_config(CAN_0_ID, &can_param);
                        if((can_param.can_mode == CAN_FD_MODE) && (can_param.data_tx_mode == DEV_XFER_INTERRUPT)){
                            snprintf(pcWriteBuffer, xWriteBufferLen, "CAN FD INT\n\r");
                        }else{
                            snprintf(pcWriteBuffer, xWriteBufferLen, "CAN INT\n\r");
                        }
                        if (DEV_XFER_INTERRUPT != can_param.data_tx_mode){
                            can_param.data_tx_mode = DEV_XFER_INTERRUPT;
                            can_set_config(CAN_0_ID,&can_param);
                        }
                        break;
                    default:
                    return pdFALSE;
                }
                    if ((NULL != param3) && (new_format >= CAN_POLLING)) {
                        if (strncmp(param3, "bk", len3) == 0) {
                            output_bk_id0 = true;
                            output_bk_id1 = true;
                            output_ak_id0 = false;
                            output_ak_id1 = false;
                            EMBARC_PRINTF("output BK data\n\r");
                        } else if (strncmp(param3, "ak", len3) == 0) {
                            output_bk_id0 = false;
                            output_bk_id1 = false;
                            output_ak_id0 = true;
                            output_ak_id1 = true;
                            EMBARC_PRINTF("output AK data\n\r");
                        } else {
                            EMBARC_PRINTF("track command data type(bk/ak) error\n\r");
                        }
                    } else if ((NULL == param3) && (new_format >= CAN_POLLING)) {
                        EMBARC_PRINTF("need to choose from bk and ak\n\r");
                    }
                    else
                    {

                    }
               }
            } else {
                EMBARC_PRINTF("track command params error\n\r");
            }
            return pdFALSE;
        } else {
                static uint32_t count = 0;
                switch(count) {
                case 0 :
                        snprintf(pcWriteBuffer, xWriteBufferLen, "new_format = %d\r\n", new_format);
                        count++;
                        return pdTRUE;
                default :
                        count = 0;
                        return pdFALSE;
                }
        }

        return pdFALSE;
}

/* track command */
static const CLI_Command_Definition_t track_cmd = {
        "track",
        "track \n\r"
        "\tWrite or read track cfg information. \n\r"
        "\tUART Usage: track [[param] value].\n\r"
        "\tCAN  Usage: track [[param] value bk/ak].\n\r",
        track_cmd_handler,
        -1
};

void track_cmd_register(void)
{
        if (track_cli_registered)
        {
                return;
        }
        FreeRTOS_CLIRegisterCommand(&track_cmd);
        track_cli_registered = true;

        /* Setting the default output mode of tracking data*/
        new_format = UART_STRING;
        return;
}

int8_t get_track_cfg(void)
{
        return new_format;
}

void set_track_cfg(int8_t value)
{
        new_format = value;
}
