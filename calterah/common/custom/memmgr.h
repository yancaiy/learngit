/*
 * @Author: fang yongjun
 * @Date: 2022-03-11 16:47:25
 * @LastEditTime: 2022-04-18 14:16:34
 * @LastEditors: fang yongjun
 * @Description: 
 * @FilePath: memmgr.h
 * Copyright (C) 2021 Chengtech Ltd.
 */
#ifndef __MEM_MGR_H__
#define __MEM_MGR_H__

#include "typedefs.h"

//MPU保护任务栈大小
#define MPU_STACK_512     (512)
#define MPU_STACK_1024    (1024)
#define MPU_STACK_2048    (2048)
#define MPU_STACK_DEFAULT MPU_STACK_512

#define SM_DATA_SEC  __attribute__((section(".mpu_data_sm")))
#define SM_BSS_SEC  __attribute__((section(".mpu_bss_sm")))

#define CFG_DATA_SEC __attribute__((section(".mpu_data_cfg")))
#define CFG_BSS_SEC __attribute__((section(".mpu_bss_cfg")))

#define CMD_FUN __attribute__((section(".cmd_func")))

extern int memMgrInit(void);
extern int unlockMem(uint32_t addr);
extern int lockMem(uint32_t addr);
extern int unlockSMDataSec(void);
extern int lockSMDataSec(void);
extern int allocMpuTaskMem(void **pStack, void **pTcb, uint32_t stackDepth);

#endif
