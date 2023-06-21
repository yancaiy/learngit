/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/**
 * \file
 * \brief	exception processing for freertos
 */
#include <string.h>
#include "embARC.h"

#include "arc_freertos_exceptions.h"
#include "embARC_assert.h"

#ifdef __GNU__
extern void gnu_printf_setup(void);
#endif
/**
 * \brief  freertos related cpu exception initialization, all the interrupts handled by freertos must be not
 * fast irqs. If fiq is needed, please install the default firq_exc_entry or your own fast irq entry into
 * the specific interrupt exception.
 */
void freertos_exc_init(void)
{

#ifdef __GNU__
	//gnu_printf_setup();
#endif

}

#define TASK_NAME_SAVE_LEN		8
uint8_t track_overflow_task_name[TASK_NAME_SAVE_LEN] = {0};

/* Check task stack overflow */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
#define UART_RAW_OUTPUT_ADDR 0x00B30000U
    if (pcTaskName != NULL) {
        /* When StackOverflow occurs, one can check the variable memory by debugger
           to know which task caused the exception  */
        memcpy(track_overflow_task_name, pcTaskName, TASK_NAME_SAVE_LEN);

        /* try to print task name by uart */
        raw_writel(UART_RAW_OUTPUT_ADDR, *pcTaskName);
        raw_writel(UART_RAW_OUTPUT_ADDR, *(pcTaskName + 1));
        raw_writel(UART_RAW_OUTPUT_ADDR, *(pcTaskName + 2));
    }

    EMBARC_ASSERT(0);
}

