
#ifndef __TARGET_PROC_H__
#define __TARGET_PROC_H__

#include "typedefs.h"
#include "cfg.h"
#include "sharedVar.h"

extern objectInfo_t   __attribute__((aligned(4))) gOutTargets[MAX_NB_OF_TARGETS * 2];  //解速度模糊后的最终输出目标
extern Target_Thres_t __attribute__((aligned(4))) gTargets[2][2][MAX_NB_OF_TARGETS];   //信号处理后的原始目标

const Target_Thres_t *getDspTargetBuf(int txProfile, int chirpType);
void resetOutTargets(int objNum);
void getRawObjListFromCan(const void *pBuf, int version);
void startTrkDbg(float trkTime);
int32_t RDP_dataProcessInit(void);
void RDP_dataProcessTask(void *param);

#endif

