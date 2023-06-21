/*
 * @Author: fang yongjun
 * @Date: 2021-11-03 11:00:37
 * @LastEditTime: 2022-09-07 14:45:24
 * @LastEditors: fang yongjun
 * @Description: 
 * @FilePath: radardsp.h
 * Copyright (C) 2021 Chengtech Ltd.
 */
#ifndef __RADARDSP_H__
#define __RADARDSP_H__
#include "typedefs.h"
#include "baseband.h"
#include "sharedVar.h"
#include "cfg.h"

#define ANT_TX0_CHIRP0_MASK 0x01
#define ANT_TX0_CHIRP1_MASK 0x02
#define ANT_TX1_CHIRP0_MASK 0x04
#define ANT_TX1_CHIRP1_MASK 0x08
#define ANT_TX0_ALL_CHIRP_MASH (ANT_TX0_CHIRP0_MASK | ANT_TX0_CHIRP1_MASK)
#define ANT_TX1_ALL_CHIRP_MASH (ANT_TX1_CHIRP0_MASK | ANT_TX1_CHIRP1_MASK)
#define ANT_ALL_CHIRP_MASK (ANT_TX0_ALL_CHIRP_MASH | ANT_TX1_ALL_CHIRP_MASH)
#define ANT_TX_CHIRP_MASK(ant, chirp)  (1 << ((chirp) * 2 + (ant)))

typedef struct
{
    uint16_t targetNum[4][2];
    uint16_t outTargets[4];
    uint16_t outAllTargets;
    uint8_t dataMode;
    uint8_t txPattern;
    uint8_t chirpType;
    uint8_t txProfile;
    uint8_t speedVagueCheck;
    uint8_t resv;
    float minimumNoise[2];    //记录最小底噪值
} radarDspParam_t;

//FMCW参数状态
enum NEW_PARAM_TYPE_E
{
    NEW_PARAM_NONE     = 0,    //只发送不做信号处理
    NEW_PARAM_SET      = 1,    //配置新的发送参数
    NEW_PARAM_CW       = 2,    //配置CW模式
    NEW_PARAM_CLOSE_TX = 3,    //关闭TX
    NEW_PARAM_ADC      = 4,    //采集ADC
    NEW_PARAM_SW_CALI  = 5,    //标定模式切换
    NEW_PARAM_NORMAL   = 0xFF, //正常处理
};

void initDspParam(void);
const radarDspParam_t *getDspParam(void);
extern int getTxSwMaskFlag(void);
extern void setTxSwMaskFlag(int val);
extern void resetTxSwMaskFlag(void);
extern bool waitDspDone(uint32_t ticksToWait);
extern void notifyDspNewFrame(void);
extern const float *getChirpTime(void);
extern uint8_t ucradardspGetChirp(void);
extern uint8_t ucradardspGetTX(void);
int32_t DSP_radarDspInit(void);


#endif

