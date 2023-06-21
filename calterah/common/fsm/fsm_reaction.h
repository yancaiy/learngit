/*
 * @Author: fang yongjun
 * @Date: 2022-03-15 19:49:46
 * @LastEditTime: 2022-04-26 16:58:04
 * @LastEditors: fang yongjun
 * @Description: 
 * @FilePath: fsm_reaction.h
 * Copyright (C) 2021 Chengtech Ltd.
 */
#ifndef __FSM_REACTION_H__
#define __FSM_REACTION_H__

enum SM_ITEM_IDX_E
{
        IDX_ITEM_SM1  ,
        IDX_ITEM_SM4  ,
        IDX_ITEM_SM5  ,
        IDX_ITEM_SM6  ,
        IDX_ITEM_SM8  ,
        IDX_ITEM_SM11 ,
        IDX_ITEM_SM12 ,
        IDX_ITEM_SM13 ,
        IDX_ITEM_SM14 ,
        IDX_ITEM_SM101,
        IDX_ITEM_SM102,
        IDX_ITEM_SM103,
        IDX_ITEM_SM104,
        IDX_ITEM_SM105,
        IDX_ITEM_SM106,
        IDX_ITEM_SM107,
        IDX_ITEM_SM108,
        IDX_ITEM_SM109,
        IDX_ITEM_SM121, 
        IDX_ITEM_SM125,
        IDX_ITEM_SM129,
        IDX_ITEM_SM130,
        IDX_ITEM_SM133,
        IDX_ITEM_SM201,
        IDX_ITEM_SM202,
        IDX_ITEM_SM203,
        IDX_ITEM_SM204,
        IDX_ITEM_SM205,
        IDX_ITEM_SM206,
        IDX_ITEM_SM207,
        IDX_ITEM_SM805,
        IDX_ITEM_SM806,
        IDX_ITEM_SM901,
        IDX_ITEM_SM902,
        IDX_ITEM_SM904,
        IDX_ITEM_SM905,
        IDX_ITEM_SM906,
        IDX_ITEM_SM907,
        IDX_ITEM_SM908,
        IDX_ITEM_SM910,
        IDX_ITEM_SM911,
        IDX_ITEM_NO_CFG,
        IDX_ITEM_CFG_CRC,
        SM_ITEM_NUM
};

extern void updateSmStatus(int itemIdx);

extern void setSmFailed(int itemIdx);

extern void updateSampleCnt(void);

extern unsigned long long getSmFailStatus(void);

#endif
