/*
 * @Author: fang yongjun
 * @Date: 2022-03-27 09:24:43
 * @LastEditTime: 2022-03-27 09:53:59
 * @LastEditors: fang yongjun
 * @Description: 
 * @FilePath: mpu_task.h
 * Copyright (C) 2021 Chengtech Ltd.
 */
#ifndef __MPU_TASK_H__
#define __MPU_TASK_H__
#include "typedefs.h"
#include "FreeRTOS.h"

TaskHandle_t createTask(TaskFunction_t pxTaskCode,
						const char * const pcName,
						const uint32_t ulStackDepth,
						void * const pvParameters,
						UBaseType_t uxPriority); 

#endif
