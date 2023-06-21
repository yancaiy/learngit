/*
 * @Author: fang yongjun
 * @Date: 2022-03-27 09:24:27
 * @LastEditTime: 2022-03-29 15:18:37
 * @LastEditors: fang yongjun
 * @Description: 
 * @FilePath: mpu_task.c
 * Copyright (C) 2021 Chengtech Ltd.
 */
#include "mpu_task.h"
#include "memmgr.h"
#include "FreeRTOS.h"

#define MPU_STACK 1

TaskHandle_t createTask(TaskFunction_t pxTaskCode,
						const char * const pcName,
						const uint32_t ulStackDepth,
						void * const pvParameters,
						UBaseType_t uxPriority)
{
    TaskHandle_t handle = NULL;

#if configSUPPORT_STATIC_ALLOCATION && MPU_STACK
    void *pTask = NULL;
    void *pTcb = NULL;

    allocMpuTaskMem(&pTask, &pTcb, ulStackDepth);
    if (pTask == NULL || pTcb == NULL)
    {
        EMBARC_PRINTF("alloc stack failed\r\n");
        return NULL;
    }

    EMBARC_PRINTF("%s stack 0x%x size %d\r\n", pcName, (uint32_t)pTask, ulStackDepth * 4);

    handle = xTaskCreateStatic(pxTaskCode,
							   pcName,
							   ulStackDepth,
							   pvParameters,
							   uxPriority,
							   (StackType_t *)pTask, (StaticTask_t *)pTcb);
    return handle;
#else
    if (pdPASS != xTaskCreate(pxTaskCode,
							 pcName,
							 ulStackDepth,
							 pvParameters,
							 uxPriority,
							 &handle))
    {
        handle = NULL;
    }

    return handle;
#endif
}