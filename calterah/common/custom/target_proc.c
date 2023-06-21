
#include <string.h>
#include <math.h>
#include "cfg.h"
#include "memmgr.h"
#include "mpu_task.h"
#include "app_dut.h"
#include "app_protocol.h"
#include "radardsp.h"
#include "radarCfg.h"
#include "app_can_msg.h"
#include "app_adc_hil.h"
#include "target_proc.h"

extern radarDspParam_t gDspParam;

static SemaphoreHandle_t gMutexTargetList = NULL; //用于track list数据拷贝过程

objectInfo_t   __attribute__((aligned(4))) gOutTargets[MAX_NB_OF_TARGETS * 2] = {0};  //解速度模糊后的最终输出目标
Target_Thres_t __attribute__((aligned(4))) gTargets[2][2][MAX_NB_OF_TARGETS] = {0};   //信号处理后的原始目标
static int gRxRawObjCnt = 0;  //通过CAN收到的目标数量
static u8 gTrkDbgMode = 0;    //上位机进行调试模式，接收can原始点数据用下位机跟踪 0表示正常模式，1表示调试模式
float gTrkDbgFrmTime;
u8 gTrkDbgFrmIdx = 0;         //调试模式下新点数据更新时候更新这个值 ++


void app_target_process(void)
{
    xSemaphoreTake(gMutexTargetList, portMAX_DELAY);

    updateTimestamp();

    if (DUTGetTestState() == DUT_MODE_CLOSE)
    {
        if(appAdcOrHilFuncFrameNumGet() == 0)
        {
            xSemaphoreGive(gMutexTargetList);

            return;
        }

        if (gDspParam.dataMode != PARAM_DATAMODE_RAW)
        {
            if (isCanValid(PCAN, gDspParam.outAllTargets * 2 + 1) == 0)
            {
                appProsendTargetsArs410(PCAN, gOutTargets, gDspParam.outAllTargets);
            }
        }
        
        // 此处只是为了让上位机显示原始点 0x80是结束帧
        char tmpBuf[8] = {0};
        gCAN[PCAN].sendFrame((void *)tmpBuf, 0x80);
    }

    xSemaphoreGive(gMutexTargetList);
}

void getRawObjListFromCan(const void *pBuf, int version)
{
    if (gTrkDbgMode == 0 || gRxRawObjCnt >= (MAX_NB_OF_TARGETS * 2))
    {
        return;
    }

    objectInfo_t *pList = &gOutTargets[gRxRawObjCnt];
    const stRawObjInfoMsg_V5 *pMsg = (const stRawObjInfoMsg_V5 *)pBuf;
    int16_t val;

    pList->isUsing = 1;
    pList->objTxCh = pMsg->TxCh;
    pList->range = pMsg->range == 0x1FFF ? 1024 : pMsg->range / 20.0;
    val = pMsg->velocity;
    val <<= 4;
    val >>= 4;
    pList->velocity = val / 20.0;
    val = pMsg->angle;
    val <<= 4;
    val >>= 4;
    pList->angle = val / 20.0;
    pList->heighAngle = pList->objTxCh == TX_CH_LONG_RANGE ? -32 : (int8_t)pMsg->elevated * 0.3;
    pList->snr = pMsg->snr / 5.0;
    pList->fftMag = pMsg->mag;

    gRxRawObjCnt++;
}

const Target_Thres_t *getDspTargetBuf(int txProfile, int chirpType)
{
    return gTargets[txProfile][chirpType];
}

void startTrkDbg(float trkTime)
{
    if (gTrkDbgMode == 0)
    {
        return;
    }

    gTrkDbgFrmTime = trkTime;
    gTrkDbgFrmIdx++;
}

uint8_t isTrkDbgMode(void)
{
    return gTrkDbgMode;
}

int32_t RDP_dataProcessInit(void)
{
    gMutexTargetList = xSemaphoreCreateBinary();
    if (gMutexTargetList == NULL)
    {
        EMBARC_PRINTF("create gMutexTargetList error\r\n");
        return E_SYS;
    }
    xSemaphoreGive(gMutexTargetList);

    /* Clear the data buffer */
    memset(&gDspParam, 0, sizeof(gDspParam));
    memset((void *)gOutTargets, 0, sizeof(gOutTargets));
    memset((void *)gTargets, 0, sizeof(gTargets));

    return E_OK;
}

void RDP_dataProcessTask(void *param)
{
    while (1)
    {
        /* 先输出前一帧的数据, 在进行跟踪处理, 保证跟踪的时间变化不影响输出的时间间隔 */
        if (gTrkDbgMode == 0)
        {
            if (waitDspDone(portMAX_DELAY))
            {
                /* Solve velocity ambiguity */
                RSP_dataProcess_fixVelocity(gTargets, &gDspParam, gOutTargets, 0, gDspParam.chirpType);
                RSP_dataProcess_fixVelocity(gTargets, &gDspParam, gOutTargets, 1, gDspParam.chirpType);

                /* Merge the RSP data process result */
                RSP_dataProcess_mergeTargets(gTargets, &gDspParam, gOutTargets);

                EMBARC_PRINTF("################# RDP_dataProcTask() #################\n");

                /* Trigger the next frame */
                notifyDspNewFrame();
            }
            else
            {
                taskYIELD();
            }
        }
        else
        {
            vTaskDelay(2);
        }
    }
}

