/**
 * @file     rsp_datapath.c
 * @brief    This file defines the functions to configure and run the dataPath of RSP(Radar Signal Process) in radar device.
 *           And use the RSP dataPath interface to get the data or output them from dataPath in RSP.
 * @author   Wison (linkangcheng@chengtech.net)
 * @version  1.0
 * @date     2022-09-27
 * 
 * 
 * @par Verion logs:
 * <table>
 * <tr>  <th>Date        <th>Version  <th>Author     <th>Description
 * <tr>  <td>2022-09-27  <td>1.0      <td>Wison      <td>First Version
 * </table>
 * @copyright Copyright (c) 2022  Shenzhen Cheng-Tech Co.,Ltd.
 */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include <math.h>
#include "app_adc_hil.h"
#include "status.h"
#include "rsp_datapath.h"

#define MAG_CALC(mag)	(uint16_t)(10 * log10f(mag) + 128)
//帧周期，单位ms
#define FRAME_CYCLE         50
#define FRAME_CYCLE_DELTA   3   //帧周期波动范围


extern fmcwParam_t gFmcwParam[TX_SW_NUM][2];
extern radarDspParam_t gDspParam;
extern QueueHandle_t gQueDataProc;
extern QueueHandle_t gQueNewFrame;

SemaphoreHandle_t mutex_frame_initial_flag;
static int32_t frameFlag;
static uint8_t gFrameTypeIdx = 0;
static Bin_Param_t gBinParam;

float gVelocityRes[2]; //速度bin精度
float gRangeRes[2];    //距离bin精度

//当前帧结束的时间，单位ms
float gFrameTime;


void DSPConfigSet(uint8_t track_frame_type, int32_t *tx_profile, int32_t *chirp_type, int32_t *tx_mask_flag)
{
    if (track_frame_type >= NUM_FRAME_TYPE) {
        return;
    }

    switch (track_frame_type)
    {
        case 0:
                *tx_profile = 0;
                *chirp_type = 0;
                *tx_mask_flag |= ANT_TX0_CHIRP0_MASK;
                break;

        case 1:
                *tx_profile = 1;
                *chirp_type = 0;
                *tx_mask_flag |= ANT_TX1_CHIRP0_MASK;
                break;

        case 2:
                *tx_profile = 0;
                *chirp_type = 1;
                *tx_mask_flag |= ANT_TX0_CHIRP1_MASK;
                break;

        case 3:
                *tx_profile = 1;
                *chirp_type = 1;
                *tx_mask_flag |= ANT_TX1_CHIRP1_MASK;
                break;

        default:
                *tx_profile = 0;
                *chirp_type = 0;
                *tx_mask_flag |= ANT_TX0_CHIRP0_MASK;
                break;
    }

    gDspParam.txProfile = *tx_profile;
    gDspParam.chirpType = *chirp_type;

    DSPConfigure(&gFmcwParam[*tx_profile][*chirp_type]);

    if (*tx_profile == 0)
    {
        gRangeRes[*chirp_type] = gBinParam.range_bin;
        gVelocityRes[*chirp_type] = gBinParam.velocity_bin;
    }
}

void DSPConfigure(const fmcwParam_t *param)
{
    float chirpUpTime = param->tup;
    int rangFftNum = param->nfft_range;
    float tchirp = param->tup + param->tdown + param->tidle;
    float adcSamFreq = ((float)(param->adcFreq))/((float)(param->decFactor));

    // range resolution, m; speed resolution, m/s
    gBinParam.range_bin = SPEED_LIGTH / 2 / (param->bandWidth * 1e9) * chirpUpTime * adcSamFreq / rangFftNum;
    gBinParam.velocity_bin = 1 / (tchirp * param->nfft_vel * 1e-6) * SPEED_LIGTH / 2 / ((param->startFrq + param->bandWidth / 2) * 1e9);
    gBinParam.angle_bin = 1;
}

Bin_Param_t getBinParam(void)
{
    return gBinParam;
}

void setFrameFlag(uint32_t data)
{
    xSemaphoreTake(mutex_frame_initial_flag, portMAX_DELAY);
    frameFlag = data;
    xSemaphoreGive(mutex_frame_initial_flag);
}

/*===========================================================================*/

float RSP_calcRcs(Target_Thres_t *pObj, int txProfile)
{
    float rcs;
//    float range = pObj->range;
//
//    //10 - 110m为有效线性范围
//    if (pObj->range < 10)
//    {
//        range = 10;
//    }
//    else if (pObj->range > 110)
//    {
//        range = 110;
//    }

    if (txProfile == 0)
    {
        rcs = 10 * log10f(pObj->fftMag) - (-33.897 - 37.3 * log10f(pObj->range/10.0)) + 0;
    }
    else
    {
        rcs = 10 * log10f(pObj->fftMag) - (-31.89 - 43.70 * log10f(pObj->range/10.0)) + 0;
    }

    if (pObj->range < 20)
    {
        rcs += 8;
    }

    if (rcs >= 63.5)
    {
        rcs = 63.5;
    }
    else if (rcs <= -63.5)
    {
        rcs = -63.5;
    }

    return rcs;
}

/*
 * 计算模糊度
 *
 * 输入：
 * velA：PRI1下的速度
 * velB：PRI2下的速度
 * maxA：PRI1下的最大不模糊速度
 * maxB：PRI2下的最大不模糊速度
 * kList:模糊系数列表
 * kLength:模糊系数长度
  *
 * 输出：
 * realVelocityA：VA的真实速度
 * realVelocityB：VB的真实速度
 * factorResultA：VA的模糊度
 * factorResultB：VB的模糊度
 * deltaVel：PRI1和PRI2下目标的速度差
 * response：是否匹配上,0：速度差太大，超过阈值
 **/
bool RSP_calcVelWrapNum(float velA, float velB, float maxA, float maxB, int kList[],int kLength, float VelRange[2],
                    float *realVelocityA, float *realVelocityB,int8_t *factorResultA, int8_t *factorResultB, float *deltaVel)
{
	*factorResultA = 0;
	*factorResultB = 0;
	*realVelocityA = velA;
	*realVelocityB = velB;
	*deltaVel = 0.0f;

	bool response = false;
	float minDeltaVel = fabsf(maxA - maxB) / 2;//对于较多的k值而言，门限可根据A-B帧最大速度确定，对于较少的k值可固定门限

	for (int i = 0; i < kLength; ++i)
	{
		float velocityA = maxA * kList[i] + velA;
		if (velocityA < VelRange[0] || velocityA > VelRange[1])
		{
			continue;
		}

		int kB = round((velocityA - velB) / maxB);
		float velocityB = maxB * kB + velB;
		if(velocityB < VelRange[0] || velocityB > VelRange[1])
			continue;

		// 计算差值
		*deltaVel = fabsf(velocityB - velocityA);
		if (*deltaVel < minDeltaVel)
		{
			*realVelocityA = velocityA;
			*realVelocityB = velocityB;
			*factorResultA = kList[i];
			*factorResultB = kB;
			response = true;
			break;
		}
	}

	return response;
}

/**
 * @brief Solve velocity ambiguity after read the object result.
 *
 * @param [in] *inTarget The input object target attribute.
 * @param [in] *dspParam The radar DSP params.
 * @param [in] tx The TX number.
 * @param [in] chirp The chirp type.
 * @param [out] *outObject The output object target attribute.
 *
 */
void RSP_dataProcess_fixVelocity(
    Target_Thres_t (*inTarget)[2][MAX_NB_OF_TARGETS],
    radarDspParam_t *dspParam,
    objectInfo_t *outObject,
    int tx,
    int chirp)
{
    int     i = 0, j = 0;
    uint8_t flag[MAX_NB_OF_TARGETS] = {0}; //速度修正标志
    Target_Thres_t *pTargets[2];
    int targetNum[2];
    int cnt = 0;
    int idx = 0;
    float maxVelocity[2]; //最大速度

    DSPConfigure(&gFmcwParam[tx][chirp]);
    gVelocityRes[chirp] = gBinParam.velocity_bin;
    maxVelocity[chirp] = gBinParam.velocity_bin * (gFmcwParam[tx][chirp].nfft_vel / 2);
    DSPConfigure(&gFmcwParam[tx][chirp ^ 1]);
    gVelocityRes[chirp ^ 1] = gBinParam.velocity_bin;
    maxVelocity[chirp ^ 1] = gBinParam.velocity_bin * (gFmcwParam[tx][chirp ^ 1].nfft_vel / 2);

    for (int k = 0; k < 1; k++)
    {
        memset(flag, 0, sizeof(flag));
        {
            if (1) // (gDspParam.txPattern)
            {
                if (tx)
                {
                    idx = MAX_NB_OF_TARGETS * tx;
                    cnt = 0; // = gDspParam.outTargets;
                }
                else
                {
                    idx = 0;
                    cnt = 0;
                }
            }

            pTargets[0] = inTarget[tx][chirp];
            pTargets[1] = inTarget[tx][chirp ^ 1];

            targetNum[0] = dspParam->targetNum[tx][chirp];
            targetNum[1] = dspParam->targetNum[tx][chirp ^ 1];
        }

        for (i = 0; i < targetNum[0]; i++)
        {
            pTargets[0][i].valid = 1;
        }

        for (i = 0; i < targetNum[1]; i++)
        {
            pTargets[1][i].valid = 1;
        }

        float VelRange[2] = {-55, 55};
        float AmaxV = maxVelocity[chirp] * 2;
        float BmaxV = maxVelocity[chirp ^ 1] * 2;
        int kstart = (int)floor(VelRange[0] / AmaxV);
        int kstop = (int)floor(VelRange[1] / AmaxV);
        int kLength = kstop - kstart + 1;
        int kList[kLength];                 // 待选模糊数列表可根据速度范围进行选择
        float maxDeltaRange = 3;
        float maxDeltaAngle = 5;
        float maxDeltaVel = fabsf(AmaxV - BmaxV) / 2;

        // 初始化模糊系数
        for (uint16_t k = 0; k < kLength; ++k)
        {
            kList[k] = kstart + k;
        }

        for (i = 0; i < targetNum[0]; i++)
        {
            // pre config target range and vel
            float raw_rng_0 = pTargets[0][i].range;
            float raw_vel_0 = pTargets[0][i].velocity;
            float raw_angle_0 = pTargets[0][i].angle;
            float minDeltaVel = 1000;
            int minIdx = -1;
            bool flag = false;
            float velocity = 0.0f;

            for (j = 0; j < targetNum[1]; j++)
            {
                // cur config target range and vel
                float raw_rng_1 = pTargets[1][j].range;
                float raw_vel_1 = pTargets[1][j].velocity;
                float raw_angle_1 = pTargets[1][j].angle;
                float deltaRange = raw_rng_1 - raw_rng_0;
                float deltaVel = raw_vel_1 - raw_vel_0;
                float deltaAngle = raw_angle_1 - raw_angle_0;

                if (deltaRange > maxDeltaRange)
                {
                    break;
                }
                if (fabsf(deltaRange) > maxDeltaRange || fabsf(deltaAngle) > maxDeltaAngle)
                {
                    continue;
                }

                // 分离车道数据
                float deltaX = pTargets[0][i].range*cosf(raw_angle_1/180*3.1415926) - pTargets[1][j].range*cosf(raw_angle_0 / 180 * 3.1415926);
                if (fabsf(raw_vel_0) < 0.2 && fabsf(raw_vel_1) < 0.2 && fabsf(deltaX) > 4)
                {
                    continue;
                }

                if (fabsf(deltaVel) > maxDeltaVel)
                {
                    float  realVelocityA;
                    float  realVelocityB;
                    float  deltaVel_ambiguity;
                    int8_t factorResultA;
                    int8_t factorResultB;
                    bool   response = RSP_calcVelWrapNum(raw_vel_0, raw_vel_1, AmaxV, BmaxV, kList, kLength, VelRange,
                                                    &realVelocityA, &realVelocityB, &factorResultA, &factorResultB, &deltaVel_ambiguity);
                    if (response)
                    {
                        flag = true;
                        if (deltaVel_ambiguity < minDeltaVel)
                        {
                            minDeltaVel = deltaVel_ambiguity;
                            minIdx = j;
                            velocity = (realVelocityA + realVelocityB) / 2;
                        }
                    }
                }
                else
                {
                    flag = true;
                    if (fabsf(deltaVel) < minDeltaVel)
                    {
                        minDeltaVel = fabsf(deltaVel);
                        minIdx = j;
                        velocity = (raw_vel_0 + raw_vel_1) / 2;
                    }
                }
            }

            if (flag == true)
            {
                sensor_config_t *pCfg_0 = sensor_config_get_config(0);
#if RANGE_VELOCITY_INTERPOLATION_FUNEN
                baseband_t *bb = baseband_get_bb(0);
                radar_sys_params_t *sys_params = &bb->sys_params;
                pTargets[0][i].range = pTargets[0][i].range - (velocity * sys_params->carrier_freq * sys_params->chirp_rampup / (sys_params->bandwidth * 1000));
#endif
                pTargets[1][minIdx].valid = 0;
                pTargets[1][minIdx].realVel = velocity;
                pTargets[0][i].realVel = velocity;

                outObject[cnt + idx].isUsing = pTargets[0][i].isUsing;
                outObject[cnt + idx].velocity = velocity;
                outObject[cnt + idx].firstvelocity = pTargets[0][i].firstvelocity;

                outObject[cnt + idx].range = pTargets[0][i].range;
                //dml情况下优先选择只有一个角度的目标的角度
                if (pTargets[0][i].angleNum == 1 && pTargets[1][minIdx].angleNum == 1)
                {
                    outObject[cnt + idx].angle = ((pTargets[0][i].angle + pTargets[1][minIdx].angle) / 2);
                }
                else if (pTargets[0][i].angleNum == 1)
                {
                    outObject[cnt + idx].angle = pTargets[0][i].angle;
                }
                else
                {
                    outObject[cnt + idx].angle = pTargets[1][minIdx].angle;
                }

                outObject[cnt + idx].snr = 10 * log10f(pTargets[0][i].fftMag) - pTargets[0][i].threshold;
                outObject[cnt + idx].rcs = RSP_calcRcs(&pTargets[0][i], tx);
                outObject[cnt + idx].fftMag = MAG_CALC(pTargets[0][i].fftMag);
                outObject[cnt + idx].doaMag = MAG_CALC(pTargets[0][i].doaMag);
                //相位
                outObject[cnt + idx].phase[0] = pTargets[0][i].phase[0];
                outObject[cnt + idx].phase[1] = pTargets[0][i].phase[1];
                outObject[cnt + idx].phase[2] = pTargets[0][i].phase[2];
                outObject[cnt + idx].phase[3] = pTargets[0][i].phase[3];

                //高度角不能直接平均
                //gOutTargets[cnt + idx].heighAngle = (pTargets[0][i].heighAngle + pTargets[1][minIdx].heighAngle) / 2;
                if (fabsf(pTargets[0][i].heighAngle) <= 28 && fabsf(pTargets[1][minIdx].heighAngle) <= 28)
                {
                    outObject[cnt + idx].heighAngle = (pTargets[0][i].heighAngle + pTargets[1][minIdx].heighAngle) / 2;
                }
                else if (fabsf(pTargets[0][i].heighAngle) <= 28)
                {
                    outObject[cnt + idx].heighAngle = pTargets[0][i].heighAngle;
                }
                else if (fabsf(pTargets[1][minIdx].heighAngle) <= 28)
                {
                    outObject[cnt + idx].heighAngle = pTargets[1][minIdx].heighAngle;
                }
                else
                {
                    outObject[cnt + idx].heighAngle = -32;
                }

                if ((fabsf(outObject[cnt + idx].range) < 12) ||
                    (fabsf(outObject[cnt + idx].angle) < (pCfg_0->track_fov_az_right - 8)))
                {   //过滤FOV边缘上距离稍远的点
                    if (++cnt == MAX_NB_OF_TARGETS)
                    {
                        break;
                    }
                }

                //多角度模式下将另一个直接保存下来
                if (pTargets[0][i].angleNum > 1 || pTargets[1][minIdx].angleNum > 1)
                {
                    memcpy(&outObject[cnt + idx], &outObject[cnt + idx - 1], sizeof(objectInfo_t));

                    if (pTargets[0][i].angleNum > 1)
                    {
                        i++;  //接下来的一个已经解了，跳一个数
                        pTargets[0][i].realVel = velocity;
                        outObject[cnt + idx].angle = pTargets[0][i].angle;
                        outObject[cnt + idx].doaMag = MAG_CALC(pTargets[0][i].doaMag);
                    }
                    else
                    {
                        outObject[cnt + idx].angle = pTargets[1][minIdx + 1].angle;
                        outObject[cnt + idx].doaMag = MAG_CALC(pTargets[1][minIdx + 1].doaMag);
                        pTargets[1][minIdx + 1].valid = 0;
                    }

                    if ((fabsf(outObject[cnt + idx].range) < 12) ||
                        (fabsf(outObject[cnt + idx].angle) < (pCfg_0->track_fov_az_right - 8)))
                    {   //过滤FOV边缘上距离稍远的点
                        if (++cnt == MAX_NB_OF_TARGETS)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    dspParam->outTargets[tx] = cnt;
}

/**
 * @brief Merge the data after track_read() and velocity ambiguity.
 *        Merge the data from gTargets to gOutTargets, or merge the original targets together.
 *
 * @param [in] *inTarget The input object target attribute.
 * @param [in] *dspParam The radar DSP params.
 * @param [out] *outObject The output object target attribute.
 *
 */
void RSP_dataProcess_mergeTargets(
    Target_Thres_t (*inTarget)[2][MAX_NB_OF_TARGETS],
    radarDspParam_t *dspParam,
    objectInfo_t *outObject)
{
    int i = 0;

    if (gProfileCount == 2)
    {
        if (radar_config_using->tx_sel == 1)           
        {
            dspParam->outAllTargets = dspParam->outTargets[0];
        }
        else
        {
            dspParam->outAllTargets = dspParam->outTargets[1];
            for (i = 0; i < dspParam->outTargets[1]; i++)
            {
                outObject[i] = outObject[MAX_NB_OF_TARGETS + i];
            }
        }
    }
    else
    {
        outObject[dspParam->outTargets[0]].range    = 1024; //分界线
        outObject[dspParam->outTargets[0]].velocity = 0;
        outObject[dspParam->outTargets[0]].angle    = 0;
        outObject[dspParam->outTargets[0]].snr      = 0;
        outObject[dspParam->outTargets[0]].mag      = 0;
        outObject[dspParam->outTargets[0]].DbfMag   = 0;
        outObject[dspParam->outTargets[0]].isUsing  = 1;    
        dspParam->outAllTargets = dspParam->outTargets[0] + dspParam->outTargets[1] + 1;
        if ((dspParam->outAllTargets) >= (MAX_NB_OF_TARGETS * 2))
        {
            dspParam->outAllTargets = (MAX_NB_OF_TARGETS * 2);
        }

        for (i = 0; i < dspParam->outTargets[1]; i++)
        {
            if ((dspParam->outTargets[0] + 1 + i) >= (MAX_NB_OF_TARGETS * 2))
            {
                break;
            }
            outObject[dspParam->outTargets[0] + 1 + i] = outObject[MAX_NB_OF_TARGETS + i];
        }
    }
}

void RSP_HilFuncDspRun(uint8_t profile)
{
    uint16_t sysEnable = 0; uint32_t event = 0;
	uint32_t index = 0;
	uint8_t funcId = ADC_HIL_FUNC_NONE;
    baseband_t *bb = NULL;
	baseband_data_proc_t* dpc = NULL;
    const sensor_config_t* cfg = NULL;
	
	static uint8_t hilTimeOutCnt = 0;
	
	if(!appCheckIfAdcOrHilFuncNeedProc())
		return;

	funcId = appAdcOrHilFuncGetFuncId();

	switch(funcId)
	{
		/******* HIL FUNC DSP PROC *******/
		case ADC_HIL_FUNC_HIL:
		{
			baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
			bb = baseband_frame_interleave_cfg(profile);
			cfg = (sensor_config_t*)(bb->cfg);
			dpc = appHilFuncBasebandDpcGet();				
			index = 0;
			
			dpc[0].sys_enable =   SYS_ENA(HIL    , true)
                           		| SYS_ENA(FFT_2D , true)
                           		| SYS_ENA(CFR    , true)
                           		| SYS_ENA(BFM    , false);

			while(!baseband_data_proc_run(&dpc[0]));

			EMBARC_PRINTF("[0]-->Hil Radar wait for frame [%d], Baseband CFG[%d]<--\n", appAdcOrHilFuncProcFrameNumGet(), profile);	

			if(baseband_wait_bb_done(INT_MODE, portMAX_DELAY) == E_OK)
			{
                if(cfg->nvarray > 1)
		        {
//		            get_all_azimuth_candidate(bb);
					dpc[1].sys_enable =   SYS_ENA(HIL    , false)
										| SYS_ENA(FFT_2D , false)
										| SYS_ENA(CFR	 , false)
										| SYS_ENA(BFM	 , true);
		            baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
					baseband_start_dpc(&dpc[1], bb);					
		            BASEBAND_ASSERT(baseband_wait_bb_done(INT_MODE, DEFAULT_TIMOUT) == E_OK);
	            }
			    
			    /* Read the detection result */
			    track_read(bb->track);

				baseband_workaroud(&bb->bb_hw);

                /* Start data process and wait for next frame to start */
				xQueueSend(gQueDataProc, (void *)bb, 0);
				xQueueReceive(gQueNewFrame, &event, portMAX_DELAY);

				//确保帧周期
				float curTime = RTC_TO_MS(timer_rtc_count());
				if (gFrameTime + FRAME_CYCLE - curTime > FRAME_CYCLE_DELTA)
				{
					vTaskDelay(pdMS_TO_TICKS(gFrameTime + FRAME_CYCLE - curTime));
				}
				gFrameTime = RTC_TO_MS(timer_rtc_count());

				hilTimeOutCnt = 0;
				EMBARC_PRINTF("[1]-->Hil Frame[%d] Done. Remaining Frame[%d]<--\n\n\n",
				        appAdcOrHilFuncProcFrameNumGet(), appAdcOrHilFuncFrameNumGet() - 1);
			}
			else
			{
				hilTimeOutCnt++;
                EMBARC_PRINTF("[1]-->Hil Frame[%d] Done. Remaining Frame[%d]<--\n\n\n",
				        appAdcOrHilFuncProcFrameNumGet(), appAdcOrHilFuncFrameNumGet() - 1);
				if(hilTimeOutCnt > 2)
				{
					chip_reset();
					while(1);
				}
			}
			appAdcOrHilFuncProcFrameNumInc();
			appAdcOrHilFuncFrameNumDec();

		    break;
		}
		default:
		    break;
	}
	return;
}

/**
 * @brief 雷达信号处理变量的初始化
 * 
 * @return int 
 *      E_OK:初始化正常
 *      其他：初始化异常
 */
int RSP_radarSignalVarInit(void)
{
    gProfileCount = 4;
    frameFlag = RADAR_INIT_FLAG_NULL;
    mutex_frame_initial_flag = xSemaphoreCreateMutex();

    return E_OK;
}

/**
 * @description: 雷达信号处理主流程，控制profile切换以及开关
 * @param {*}
 * @return {*}
 */
void RSP_radarSignalProcessTask(void)
{
    gFrameTime = RTC_TO_MS(timer_rtc_count());

    while (1)
    {
		if(appCheckIfAdcOrHilFuncNeedProc())
		{
			appAdcOrHilFuncWorkStateSwitchProc();
		}
		else
		{
			vTaskDelay(1);

			continue;
		}

		if(appCheckIfAdcOrHilFuncNeedProc())
		{
			RSP_HilFuncDspRun(gFrameTypeIdx);
			gFrameTypeIdx = (gFrameTypeIdx + 1) % gProfileCount;
		}
		else
		{
			continue;			
		}
    }
}

