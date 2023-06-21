/*
 * @Author: fang yongjun
 * @Date: 2021-11-04 10:49:35
 * @LastEditTime: 2022-05-14 09:35:35
 * @LastEditors: fang yongjun
 * @Description: CAN传输模块
 * @FilePath: can_trans.h
 * Copyright (C) 2021 Chengtech Ltd.
 */
#ifndef __CAN_TRANS_H__
#define __CAN_TRANS_H__
#include "typedefs.h"
#include "can_hal.h"

#define CAN_RX_PKG_LEN 64

enum TX_MSG_EVENT_E
{
    TX_MSG_EVENT_START,
    TX_MSG_EVENT_TICK,
    TX_MSG_EVENT_RESET_0,
    TX_MSG_EVENT_RESET_1,
};

typedef struct
{
    uint32_t id;
    unsigned char data[eDATA_LEN_64];
    uint8_t dlc;
} stCanTxMsg;

typedef struct tagCANRxBuf
{
    uint32_t id;
    unsigned char data[CAN_RX_PKG_LEN];
    uint8_t dlc;
    uint8_t resv[3];
} TCANRxBuf;

typedef struct tagCANCtrl
{
    int (*sendFrame)(void *buf, uint32_t ID);      //通过此接口发送的报文会做字节序转换
    void (*sendFrameNow)(void *buf, uint32_t ID, uint8_t len);  //通过此接口发送的报文不会做字节序转换
} TCANCtrl;

extern const TCANCtrl gCAN[];

void setCanTxEnMask(uint8_t mask);
int isCanValid(int32_t canIdx, int cnt);
int canTransInit(void);
void canStartSend(void);
void convert(uint8_t *ary);
int canReInit(int id);
int getCanMode(void);
void canRxMsgProc(can_data_message_t *pMsg);
int agingInfoSendEn(void);

int getCanTxQue(int canCh, TickType_t waitTime, stCanTxMsg **pMsg);
int putCanTxQue(int canCh, int msgNum);

int getCanWorkMode(void);
void setCanWorkMode(int mode);

bool isBusoff(int canIdx);

#endif
