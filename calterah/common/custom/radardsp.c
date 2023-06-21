
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cfg.h"
#include "radarCfg.h"
#include "track.h"
#include "can_dbg.h"
#include "typedefs.h"
#include "apb_lvds.h"
#include "radio_ctrl.h"
#include "sensor_config.h"
#include "baseband.h"
#include "baseband_dpc.h"
#include "baseband_hw.h"
#include "arc_timer.h"
#include "func_safety.h"
#include "fsm_reaction.h"
#include "status.h"
#include "app_dut.h"
#include "app_can_msg.h"
#include "app_vehicle.h"
#include "app_adc_hil.h"
#include "target_proc.h"
#include "radardsp.h"


extern float gHILPara[3];       //0:车速；1：帧间隔；2：yawrate;

//信号处理完成消息队列
QueueHandle_t gQueDataProc = NULL;
//新一轮信号处理消息队列
QueueHandle_t gQueNewFrame = NULL;
//profile切换mask
static int gTxSwMaskFlag = -1;
//雷达信号处理参数
radarDspParam_t gDspParam;

const radarDspParam_t *getDspParam(void)
{
    return &gDspParam;
}

void initDspParam(void)
{
    gDspParam.dataMode = PARAM_DATAMODE_TRACK;
	gDspParam.targetNum[0][0] = 0;
	gDspParam.targetNum[0][1] = 0;
	gDspParam.targetNum[1][0] = 0;
	gDspParam.targetNum[1][1] = 0;
	gDspParam.outTargets[0] = 0;
	gDspParam.outTargets[1] = 0;
	gDspParam.outAllTargets = 0;
	gDspParam.txPattern = radar_config_using->tx_pattern;
	gDspParam.speedVagueCheck = SPEED_VAGUE_CHECK_EN;
    gDspParam.minimumNoise[0] = -90;
    gDspParam.minimumNoise[1] = -90;
}

int getTxSwMaskFlag(void)
{
    return gTxSwMaskFlag;
}

void setTxSwMaskFlag(int val)
{
    gTxSwMaskFlag = val;
}

void resetTxSwMaskFlag(void)
{
    gTxSwMaskFlag = -1;
}

static void updateTxSwMaskFlag(int profile)
{
    if (gTxSwMaskFlag < 0)
    {
        gTxSwMaskFlag = (~radar_config_using->tx_pattern) & ANT_ALL_CHIRP_MASK;
        memset(gDspParam.targetNum, 0, sizeof(gDspParam.targetNum));
        memset(gDspParam.outTargets, 0, sizeof(gDspParam.outTargets));

        return;
    }

    gTxSwMaskFlag |= (1 << profile);
}

/**
 * @description: 等待信号处理完成
 * @param {uint32_t} ticksToWait 等待超时时间，单位tick
 * @return {*} true - 成功，false - 超时
 */
bool waitDspDone(uint32_t ticksToWait)
{
    uint32_t event;

    return xQueueReceive(gQueDataProc, &event, ticksToWait) == pdTRUE ? true : false;
}

/**
 * @description: 通知信号处理线程开始新一帧处理
 * @param {*}
 * @return {*}
 */
void notifyDspNewFrame(void)
{
    uint32_t event = 0;

    xQueueSend(gQueNewFrame, &event, 0);
}

float getGlobalNoise(baseband_hw_t *bb_hw, int nvarray, int rng_nfft, int vel_nfft)
{
    float noise_power = 0;
    float noise_power1 = 0;
    float noise_power2 = 0;

    uint16_t cnt = 0;
    uint16_t cnt1, cnt2;
    uint8_t  bpm_idx_min = 0;
    uint8_t  bpm_idx_max = 0;// bpm_idx_max强制为0，只使用4个RX通道数据
    uint32_t hw_vel_index2;

    uint32_t noiseRegionRangeStart;
    uint32_t noiseRegionRangeEnd;

    uint32_t noiseRegion1VelocityStart;
    uint32_t noiseRegion1VelocityEnd;
    uint32_t noiseRegion2VelocityStart;
    uint32_t noiseRegion2VelocityEnd;
    uint32_t velocityRegionStep;

    noiseRegionRangeStart = (uint32_t)roundf(0.375 * rng_nfft);
    noiseRegionRangeEnd = (uint32_t)roundf(0.4 * rng_nfft);

    velocityRegionStep = (uint32_t)roundf(vel_nfft / 6);
    noiseRegion1VelocityStart = velocityRegionStep;
    noiseRegion1VelocityEnd = (uint32_t)roundf(vel_nfft / 2 - velocityRegionStep);

    uint32_t old1 = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
    for (uint32_t hw_rng_index = noiseRegionRangeStart; hw_rng_index < noiseRegionRangeEnd; hw_rng_index++)
    {
        for (uint32_t hw_vel_index1 = noiseRegion1VelocityStart; hw_vel_index1 < noiseRegion1VelocityEnd; hw_vel_index1++)
        {
            cnt1 = 0;
            for (uint8_t bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) 
            {
                for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++)
                {
                    uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, hw_rng_index, hw_vel_index1, 1);
                    complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);
                    noise_power1 += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i); // Non-coherent accumulation of 4 channel 2D-FFT power
                    cnt1++;
                }
            }
            noise_power1 = noise_power1 / cnt1;

            cnt2 = 0;
            hw_vel_index2 = (uint32_t)roundf(hw_vel_index1 + vel_nfft / 2);
            for (uint8_t bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) 
            {
                for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++)
                {
                    uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, hw_rng_index, hw_vel_index2, 1);
                    complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);
                    noise_power2 += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i); // Non-coherent accumulation of 4 channel 2D-FFT power        
                    cnt2++;
                } 
            }
            noise_power2 = noise_power2 / cnt2;

            if(noise_power1 < noise_power2)
            {
                noise_power = noise_power + noise_power1;
            }
            else
            {
                noise_power = noise_power + noise_power2;
            }
            cnt++;
        }
    }

    noise_power = noise_power / cnt;

    baseband_switch_mem_access(bb_hw, old1);

    return 10 * log10f(noise_power);
}

int32_t DSP_radarDspInit(void)
{
    gQueDataProc = xQueueCreate(1, sizeof(uint32_t));
    gQueNewFrame = xQueueCreate(1, sizeof(uint32_t));

    if (gQueDataProc == NULL || gQueNewFrame == NULL)
    {
        EMBARC_PRINTF("Create Queue failed\r\n");
        return E_SYS;
    }

    gTxSwMaskFlag = ANT_ALL_CHIRP_MASK;
    if (radar_config_using->tx_pattern & TX1_C0_C1)
    {
        gTxSwMaskFlag &= ~ANT_TX_CHIRP_MASK(0, 0);
    }
    
    if (radar_config_using->tx_pattern & TX2_C0_C1)
    {
        gTxSwMaskFlag &= ~ANT_TX_CHIRP_MASK(1, 0);
    }

    initDspParam();

    return E_OK;
}

