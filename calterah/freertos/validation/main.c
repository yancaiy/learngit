/* ------------------------------------------
 * Copyright (c) 2017, Synopsys, Inc. All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1) Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.

 * 3) Neither the name of the Synopsys, Inc., nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
--------------------------------------------- */

/**
 * \defgroup    EMBARC_APP_FREERTOS_KERNEL  embARC FreeRTOS Kernel Example
 * \ingroup     EMBARC_APPS_TOTAL
 * \ingroup     EMBARC_APPS_OS_FREERTOS
 * \brief       embARC Example for testing FreeRTOS task switch and interrupt/exception handling
 *
 * \details
 * ### Extra Required Tools
 *
 * ### Extra Required Peripherals
 *
 * ### Design Concept
 *     This example is designed to show the functionality of FreeRTOS.
 *
 * ### Usage Manual
 *     Test case for show how FreeRTOS is working by task switching and interrupt/exception processing.
 *     ![ScreenShot of freertos-demo under freertos](pic/images/example/emsk/emsk_freertos_demo.jpg)
 *
 * ### Extra Comments
 *
 */

/**
 * \file
 * \ingroup     EMBARC_APP_FREERTOS_KERNEL
 * \brief       main source file of the freertos demo
 */

/**
 * \addtogroup  EMBARC_APP_FREERTOS_KERNEL
 * @{
 */

#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"
#include "command.h"
#include "baseband.h"
#include "baseband_cli.h"
#include "radio_ctrl.h"
#include "radio_reg.h"
#include "sensor_config.h"
#include "sensor_config_cli.h"
#include "baseband_task.h"
#include <stdlib.h>

#ifdef USE_IO_STREAM
#include "data_stream.h"
#endif

#include "flash.h"
#include "gpio_hal.h"

#include "console.h"
#include "can_cli.h"
#include "can_signal_interface.h"
#include "common_cli.h"
#include "spi_master.h"
#include "apb_lvds.h"

#ifdef CHIP_CASCADE
#include "cascade.h"
#endif

#ifdef SPIS_SERVER
#include "spis_server.h"
#endif

#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif

#ifdef CPU_CLOCK_LOCK_DETECTOR
#include "cpu_clock_lock_detector.h"
#endif

#ifdef CPU_STAT
#include "statistic.h"
#endif

#define TSK_PRIOR_HI        (configMAX_PRIORITIES-1)
#define TSK_PRIOR_LO        (configMAX_PRIORITIES-2)

/* Task IDs */

/* Task Communication */
QueueHandle_t queue_cas_sync;
QueueHandle_t queue_spi_cmd;
QueueHandle_t queue_fix_1p4;
SemaphoreHandle_t mutex_frame_count;
SemaphoreHandle_t mutex_initial_flag;
EventGroupHandle_t bb_int_event_handle;

/* baseband */

/**
 * \brief  call FreeRTOS API, create and start tasks
 */
int main(void)
{
#if ACC_BB_BOOTUP == 0
        EMBARC_PRINTF("Benchmark CPU Frequency: %d Hz\r\n", get_current_cpu_freq());
#endif
        vTaskSuspendAll();

        bb_int_event_handle = xEventGroupCreate();
        /* TODO: This validation command line is not based Console and FreeRTOS Cli */
        // validation_os_entry();

        /* Init Command line */
        freertos_cli_init();

        /* Init UART console */
        console_init();

        /* Register command lines for validation test */
        common_cmd_init();

        /* Set the interrupt threshold in the STATUS32 register to 0xf.
         * It is used to enable interrupt processing.
         * */
        EMBARC_PRINTF("[main] interrupt will be enable!\r\n");
        cpu_unlock_restore(0xff);

        EMBARC_PRINTF("[main] OS will start!\r\n");
        xTaskResumeAll();

        vTaskSuspend(NULL);
        vTaskSuspend(NULL);

        return 0;
}

/** @} */
