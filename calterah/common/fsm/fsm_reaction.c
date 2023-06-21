/*
 * @Author: fang yongjun
 * @Date: 2022-03-15 19:49:25
 * @LastEditTime: 2022-05-10 10:10:21
 * @LastEditors: fang yongjun
 * @Description: 功能安全触发后的处理机制实现
 * @FilePath: fsm_reaction.c
 * Copyright (C) 2021 Chengtech Ltd.
 */
#include "fsm_reaction.h"
#include "typedefs.h"
#include "memmgr.h"
#include "status.h"
#include "radardsp.h"

//功能安全失败次数最大值
#define MAX_FAIL_CNT 0xFF

//======================== MPU保护的变量 ======================================
//功能安全失败次数
SM_DATA_SEC uint8_t gSmFailCnt[SM_ITEM_NUM] = {0, };

//功能安全失败后恢复正常次数
SM_DATA_SEC uint8_t gSmRecoverCnt[SM_ITEM_NUM] = {0, };

//功能安全实时状态掩码，1表示异常
SM_DATA_SEC uint64_t gSmStatusMask = 0;

//功能安全故障状态掩码，1表示异常
SM_DATA_SEC uint64_t gSmFaultMask = 0;

//功能安全发生过的异常状态掩码，1表示异常，主要用于测试查询
uint64_t gSmHisFailMask = 0;

//FMCW采样次数
SM_DATA_SEC uint32_t gSampleCnt = 0;

//功能安全复位系统状态
SM_DATA_SEC bool gSmReset = false;

//功能安全静默状态
SM_DATA_SEC int8_t gSmSilent = 0;

//接收饱和次数
SM_DATA_SEC int8_t gRxSatNum = 0;
//=============================================================================

static void smSilent(int itemIdx)
{
    unlockSMDataSec();

    gSmSilent++;

    lockSMDataSec();

    setSysSilent(true);
}

static void smReset(int itemIdx)
{
    if (!gSmReset)
    {
        unlockSMDataSec();

        gSmFaultMask |= (uint64_t)1 << itemIdx;
        
        gSmReset = true;

        lockSMDataSec();

        setSysReset();
    }
}

static void smReaction(int itemIdx, int silentThrd, int resetThrd)
{
    if (gSmFailCnt[itemIdx] >= resetThrd)
    {
        smReset(itemIdx);
    }
    else if (gSmFailCnt[itemIdx] == silentThrd)
    {
        unlockSMDataSec();

        gSmFaultMask |= (uint64_t)1 << itemIdx;

        lockSMDataSec();

        smSilent(itemIdx);
    }
    else
    {
        
    }
}

//RX饱和处理
static void rxSaturation(void)
{
    bool overtime = false;

    unlockSMDataSec();

    if (++gRxSatNum >= 2)
    {
        gRxSatNum = 0;
        overtime = true;
    }

    lockSMDataSec();

//    if (overtime)
//    {
//        rxSatProc();
//    }
}

void setSmFailed(int itemIdx)
{
    unlockSMDataSec();
    
    gSmStatusMask |= ((uint64_t)1 << itemIdx);
    gSmHisFailMask |= ((uint64_t)1 << itemIdx);

    if (gSmFailCnt[itemIdx] < MAX_FAIL_CNT)
    {
        gSmFailCnt[itemIdx]++;
    }

    if (gSmRecoverCnt[itemIdx] > 0)
    {
        gSmRecoverCnt[itemIdx] = 0;
    }

    lockSMDataSec();

    switch (itemIdx)
    {
        case IDX_ITEM_SM1:
        case IDX_ITEM_SM5:
        case IDX_ITEM_SM101:
        case IDX_ITEM_SM102:
        case IDX_ITEM_SM103:
        case IDX_ITEM_SM104:
        case IDX_ITEM_SM105:
        case IDX_ITEM_SM106:
        case IDX_ITEM_SM107:
        case IDX_ITEM_SM108:
        case IDX_ITEM_SM109:
        case IDX_ITEM_SM204:
        case IDX_ITEM_SM206:
        case IDX_ITEM_SM133:
        case IDX_ITEM_SM130:
        case IDX_ITEM_SM805:
        case IDX_ITEM_SM901:
        case IDX_ITEM_SM902:
        case IDX_ITEM_SM904:
        case IDX_ITEM_SM905:
        case IDX_ITEM_SM906:
        case IDX_ITEM_SM907:
        case IDX_ITEM_SM908:
        case IDX_ITEM_SM910:
        case IDX_ITEM_SM911:
            smReset(itemIdx);
            break;

        case IDX_ITEM_SM4:
        case IDX_ITEM_SM6:
        case IDX_ITEM_SM11:
        case IDX_ITEM_SM12:
        case IDX_ITEM_SM13:
        case IDX_ITEM_SM201:
            smReaction(itemIdx, 3, 20);
            break;

        case IDX_ITEM_SM8:
            rxSaturation();
            break;

        case IDX_ITEM_SM129:
            //TODO reg readback
            break;

        case IDX_ITEM_NO_CFG:
        case IDX_ITEM_CFG_CRC:
            smReaction(itemIdx, 1, 2);
            break;
    }
}

static void setSmRecover(int itemIdx)
{
    switch (itemIdx)
    {
        case IDX_ITEM_SM4:
        case IDX_ITEM_SM6:
        case IDX_ITEM_SM11:
        case IDX_ITEM_SM12:
        case IDX_ITEM_SM13:
        case IDX_ITEM_SM201:
            if (gSmFailCnt[itemIdx] >= 3)
            {
                gSmSilent--;

                if (gSmSilent == 0)
                {
                    setSysSilent(false);
                }
            }

            break;

        case IDX_ITEM_SM129:
            //TODO reg readback
            break;

    }
}

static void updateFailCnt(int itemIdx)
{
    switch (itemIdx)
    {
        case IDX_ITEM_SM4:
        case IDX_ITEM_SM6:
        case IDX_ITEM_SM8:
        case IDX_ITEM_SM11:
        case IDX_ITEM_SM12:
        case IDX_ITEM_SM13:
        case IDX_ITEM_SM201:
            if (++gSmRecoverCnt[itemIdx] >= 2)
            {
                setSmRecover(itemIdx);

                gSmFailCnt[itemIdx] = 0;
                gSmRecoverCnt[itemIdx] = 0;
            }

            break;
            
        default:
            break;
    }

    if (gSmFailCnt[itemIdx] == 0)
    {
        gSmStatusMask &= ~((uint64_t)1 << itemIdx);
        gSmFaultMask &= ~((uint64_t)1 << itemIdx);
    }
}

void updateSampleCnt(void)
{
    unlockSMDataSec();
        
    gSampleCnt++;

    updateFailCnt(IDX_ITEM_SM4);
    updateFailCnt(IDX_ITEM_SM8);

    // for (int i = 0; i < SM_ITEM_NUM; i++)
    // {
    //     if (gSmFailCnt[i] > 0)
    //     {
    //         updateFailCnt(i);
    //     }
    // }

    lockSMDataSec();
}

uint64_t getSmFailStatus(void)
{
    return gSmFaultMask;
}

void updateSmStatus(int itemIdx)
{
    unlockSMDataSec();

    updateFailCnt(itemIdx);

    lockSMDataSec();
}