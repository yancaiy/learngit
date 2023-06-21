#ifndef __APP_ADC_HIL_H__
#define __APP_ADC_HIL_H__

#include <typedefs.h>
#include "baseband_dpc.h"


#define ADC_HIL_PROFILE_MAX_NUM 4

/**************** ADC_DUMP & HIL Function *****************/
typedef enum __HIL_MODE_PARAM__{
	HIL_FUNC_MODE_NULL  = 0, //
	HIL_FUNC_MODE_FFT1D = 1, //must be equal to DMP_FFT_1D
	HIL_FUNC_MODE_FFT2D = 2, //must be equal to DMP_FFT_2D	
}hilFuncMode;

typedef enum __ADC_OR_HIL_FUNC_ID__{
	ADC_HIL_FUNC_ADC = 0,
	ADC_HIL_FUNC_HIL = 1,
	ADC_HIL_FUNC_NONE = 2,
}appAdcOrHilId;

typedef enum __ADC_OR_HIL_FUNC_STATE__{
	ADC_HIL_FUNC_STOP = 0,
	ADC_HIL_FUNC_RUNNING = 1,
	ADC_HIL_FUNC_START = 2,
}appAdcOrHilState;

enum __ADC_OR_HIL_FUNC_STATE_SCHEDULE__{
	ADCHIL_STATU_NO_CHANGE = 0,
	ADCHIL_STATU_SW_TO_RUNNING = 1,
	ADCHIL_STATU_SW_TO_STOP = 2,
};

typedef struct  __ADC_OR_HIL_FUNC_PARAMS__{
	appAdcOrHilId funcId;
	appAdcOrHilState workState;
	volatile uint32_t frameNum;
	volatile uint32_t procFrameNum; //hil processed frame num, just for print info 1~frameNum
	uint8_t param; /*adcDump[0:normal 1:tone] hilFunc[0:NULL 1:FFT1D 2:FFT2D]*/
	uint8_t profileIdx;
	uint8_t profileNum;
	uint8_t sysEnable; 
	baseband_data_proc_t dpcForHil[3];
	float hilFrameTime;
	uint32_t hilFrameInterval;
}appAdcOrHilParams;

extern int32_t appAdcOrHilFuncParamSetupProc(appAdcOrHilId funcId, uint32_t frameNum, uint8_t param);
bool appCheckIfAdcOrHilFuncNeedProc(void);
uint32_t appAdcOrHilFuncWorkStateSwitchProc(void);
extern void appHilFuncFrameIntervalSet(uint32_t frameInterval);
appAdcOrHilId appAdcOrHilFuncGetFuncId(void);
void appAdcOrHilFuncFrameNumDec(void);
uint32_t appAdcOrHilFuncFrameNumGet(void);
void appAdcOrHilFuncProcFrameNumInc(void);
uint32_t appAdcOrHilFuncProcFrameNumGet(void);
baseband_data_proc_t *appHilFuncBasebandDpcGet(void);


#endif

