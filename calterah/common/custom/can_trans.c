/*
 * @Author: fang yongjun
 * @Date: 2021-11-04 10:49:25
 * @LastEditTime: 2022-09-05 12:02:28
 * @LastEditors: fang yongjun
 * @Description: CAN传输模块
 * @FilePath: can_trans.c
 * Copyright (C) 2021 Chengtech Ltd.
 */
#include <string.h>
//#include "mpu_task.h"
#include "FreeRTOS.h"
#include "can_trans.h"
#include "radarCfg.h"
#include "cfg.h"
#include "sharedVar.h"
#include "app_dut.h"
#include "app_protocol.h"
#include "radardsp.h"
//#include "target_proc.h"
//#include "diagnostic.h"
//#include "uds_network_layer.h"
#include "can_dbg.h"
#include "can_hal.h"
#include "dev_can.h"
#include "status.h"
#include "memmgr.h"


#define CAN_1000K_DIV           3
#define CAN_1000K_DELAYTIME     125
#define CAN_500K_DIV            7
#define CAN_500K_DELAYTIME      250

#define CAN_DIV             CAN_500K_DIV
#define CAN_DELAYTIME       CAN_500K_DELAYTIME

#define CAN_TX_BUF_SIZE     100

#define BUSOFF_TEST_PERIOD  5       // ms
#define BUSOFF_QUICK_TIME   50      //ms
#define BUSOFF_SLOW_TIME    200     //ms
#define BUSOFF_QUICK_CNT    5       // 慢恢复测试，大于这个数后变为快恢复，如果与BUSOFF_ALL_CNT相对，那就没有慢恢复
#define BUSOFF_ALL_CNT      8       //  设置最大值，busoff到这个值会一直保持，这个值需要比实际dtc检测的次数大于等于

typedef struct
{
    uint8_t start;
    uint8_t now;   //正在发送立即目标, 0 - 没有, 1 - 待发送，2 - 正在发送
    uint16_t num;
    uint16_t idx;
    uint16_t sendCnt;
    stCanTxMsg msgBuf[CAN_TX_BUF_SIZE];  //消息buf
    stCanTxMsg msgNow;                   //需要马上发送的目标
    xSemaphoreHandle sem;
} TxTargets_t;

static void startSendMsg(int canIdx);

//是否使用canfd模式
static int gCanFdMode = 0;
//发送线程消息队列
static QueueHandle_t gQueTxMsg;
//发送任务句柄
static TaskHandle_t gTxTaskHandle;
//发送队列
volatile TxTargets_t gTxTargets[2];
//can发送使能掩码，每个bit代表一路can
static uint8_t gCanTxEnMask = CAN_TX_EN_MASK;
//CAN发送定时器
static xTimerHandle xTimerTrackListSend;
//busoff处理定时器
static xTimerHandle xTimerCan0Busoff;

static int can0_busoff_recover_timer = 0;
static int can1_busoff_recover_timer = 0;
static int can0_busoff_timer = 0;
static int can1_busoff_timer = 0;
static int can0_busoff_cnt = 0;
static int can1_busoff_cnt = 0;
static volatile int8_t can0_busoff_flag = 0;
static volatile int8_t can1_busoff_flag = 0;

//busoff保持期间，此时不发送任何数据
static bool gCanBusOffHold[2] = {false, false};

int gCanBusWorkMode = 0; //0 - 正常 ，1、屏蔽模式

int getCanWorkMode(void)
{
    return gCanBusWorkMode;
}

void setCanWorkMode(int mode)
{
    gCanBusWorkMode = mode;
}

int getBusoffCnt(int canIdx)
{
    return canIdx == CAN_0_ID ? can0_busoff_cnt : can1_busoff_cnt;
}

bool isBusoff(int canIdx)
{
    return gCanBusOffHold[canIdx] || (canIdx == CAN_0_ID ? can0_busoff_flag : can1_busoff_flag);
}

/**
 * @description: 设置CAN发送使能掩码
 * @param {uint8_t} mask
 * @return {*}
 */
void setCanTxEnMask(uint8_t mask)
{
    gCanTxEnMask = mask;
}

int agingInfoSendEn(void)
{
    if (gCanBusWorkMode != CAN_BUS_WORK_MODE_ALL_Shield
		&& radar_config_using->agingTest == DUT_AGINT_TEST_EN )
	{
		//发送
		return 1;
	}
	else
	{
		//不发送
		return 0;
	}
}

void can0ErrorCallback(uint32_t errorStat)
{
    if (errorStat & CAN_INT_BUS_OFF)
    {
        can0_busoff_flag = 1;
    }
}

void can1ErrorCallback(uint32_t errorStat)
{
    if (errorStat & CAN_INT_BUS_OFF)
    {
        can1_busoff_flag = 1;
    }
}

int canReInit(int canIdx)
{
    int result =E_OK;

    can_config_t config;

    can_get_config(canIdx, &config);
    if( CAN_0_ID == canIdx )
    {
        config.nomi_baud = RADAR_CAN0_NOMI_BAND;
        config.data_baud = RADAR_CAN0_DATA_BAND;
    }
    else
    {
        config.nomi_baud = RADAR_CAN1_NOMI_BAND;
        config.data_baud = RADAR_CAN1_DATA_BAND;
    }
    can_set_config(canIdx, &config);

    can_bus_off_recover(canIdx);
    gTxTargets[canIdx].now = 0;
    
	return result;
}

//等待之前的发送完成，如果长时间没有发送成功则认为CAN有错误，这里会一直等待发送完成
int isCanValid(int32_t canIdx, int cnt)
{
	volatile TxTargets_t *txTargets = &(gTxTargets[canIdx]);

    int retry = 2;
    int idx = txTargets->idx;
    uint32_t event = TX_MSG_EVENT_RESET_0;

    if (txTargets->num + cnt > CAN_TX_BUF_SIZE || isBusoff(canIdx))
    {
        //EMBARC_PRINTF("0 isCanValid %d, num %d\n", txTargets->num, cnt);
        return -2;
    }

    while (txTargets->start)
    {
        vTaskDelay(2);

        if (txTargets->start == 0)
        {
            break;
        }

        if (idx == txTargets->idx)
        {
            if (retry-- < 1)
            {
                // EMBARC_PRINTF("wait can send failed\r\n");
                return -1;
            }
        }
        else
        {
            idx = txTargets->idx;
        }
    }

    return 0;
}

void setCanMode(int fd)
{
    gCanFdMode = fd;
}

int getCanMode(void)
{
	return gCanFdMode;
}

void convert(uint8_t *ary)
{
    uint8_t tmp;
    tmp = ary[0];
    ary[0] = ary[7];
    ary[7] = tmp;
    tmp = ary[1];
    ary[1] = ary[6];
    ary[6] = tmp;
    tmp = ary[2];
    ary[2] = ary[5];
    ary[5] = tmp;
    tmp = ary[3];
    ary[3] = ary[4];
    ary[4] = tmp;
}

void convert_ping_pong(uint8_t *ary, uint8_t len)
{
    uint8_t times = len / 2;
    uint8_t tmp;
    uint8_t i;

    for (i = 0; i < times; i++)
    {
        tmp = ary[i];
        ary[i] = ary[len-i-1];
        ary[len-i-1] = tmp;
    }

	uint64_t ultemp;
	uint64_t *pData = (uint64_t*)ary;
	uint8_t Indexmax = (len+7)/8;
	for(uint8_t j=0; j<Indexmax/2 ; j++)
	{
		ultemp = pData[j];
		pData[j] = pData[Indexmax-j-1];
		pData[Indexmax-j-1] = ultemp;
	}
}

static void can_tx_callback(int canIdx)
{
    int result;
    int cnt;
    uint32_t event = canIdx == CAN_0_ID ? TX_MSG_EVENT_RESET_0 : TX_MSG_EVENT_RESET_1;
    volatile TxTargets_t *pTargets = &gTxTargets[canIdx];

    if (pTargets->now == 1)
    {
        result = can_send_data_isr(canIdx, pTargets->msgNow.id, (uint32_t *)pTargets->msgNow.data, eDATA_LEN_8);
        if (result != E_OK)
        {
            EMBARC_PRINTF("can_send_data_isr result %d\r\n", result);
        }

        pTargets->now = 2;
        
        return;
    }

    if (pTargets->now == 2)
    {
        pTargets->now = 0;

        //只发送需要发送的消息
        if (pTargets->start == 0)
        {
            return;
        }
    }

    if (pTargets->start)
    {
        pTargets->idx += pTargets->sendCnt;
        pTargets->sendCnt = 0;
    }
    
    if (pTargets->start == 1 && pTargets->idx < pTargets->num /*&& !isSilent()*/)
    {
        pTargets->sendCnt = 1;
        
        if( 0 == pTargets->msgBuf[pTargets->idx].dlc )
        {
            pTargets->msgBuf[pTargets->idx].dlc = eDATA_LEN_8;
        }

        uint8_t data[64];
        memcpy(data, (uint8_t*)(pTargets->msgBuf[pTargets->idx].data), pTargets->msgBuf[pTargets->idx].dlc);
        convert_ping_pong((uint8_t *)(data), pTargets->msgBuf[pTargets->idx].dlc);

        result = can_send_data_isr(canIdx, pTargets->msgBuf[pTargets->idx].id,
                                   (uint32_t *)data,
                                   pTargets->msgBuf[pTargets->idx].dlc);
        if (result != E_OK)
        {
            EMBARC_PRINTF("can_send_data_isr result %d\r\n", result);
        }
        else
        {
            //pTargets->idx++;
        }
    }
    else if (pTargets->start == 0 /*|| isSilent()*/ || uxSemaphoreGetCount(pTargets->sem))
    {
        pTargets->num = 0;
        pTargets->idx = 0;
        pTargets->start = 0;
        pTargets->sendCnt = 0;
    }
    else
    {
        
    }
}

static void can0TxCallBack(uint32_t msgId, uint32_t txFailed)
{
    can_tx_callback(CAN_0_ID);
}

static void can1TxCallBack(uint32_t msgId, uint32_t txFailed)
{
    can_tx_callback(CAN_1_ID);
}

static void can_send_async(int canIdx, int idx)
{
    //can_interrupt_enable(canIdx, 1, 0);

    int result;
    int retry = 0;
    volatile TxTargets_t *pTargets = &(gTxTargets[canIdx]);

    int cnt = pTargets->sendCnt;
    if (cnt == 0)
    {
        pTargets->sendCnt = 1;
    }

    if( 0 == pTargets->msgBuf[idx].dlc )
    {
        pTargets->msgBuf[idx].dlc = eDATA_LEN_8;
    }

    uint8_t data[64];
    memcpy(data, (uint8_t*)(pTargets->msgBuf[idx].data), pTargets->msgBuf[idx].dlc);
	convert_ping_pong((uint8_t *)(data), pTargets->msgBuf[idx].dlc);

    result = can_send_data(canIdx, pTargets->msgBuf[idx].id, (uint32_t *)data, pTargets->msgBuf[idx].dlc);
    if (result != E_OK)
    {
        pTargets->sendCnt = 0;
        //EMBARC_PRINTF("can_send_data %d ret %d\r\n", canIdx, result);
    }
    else
    {
        //pTargets->idx++;
    }

    //can_interrupt_enable(canIdx, 1, 1);
}

/* FreeRTOS timer callback */
static void vTimerCan0SendCallback(xTimerHandle xTimer) //50ms软件中断
{
#if TRK_INTERPOLATION
    if (isCfgMode() || DUTGetTestState() != DUT_MODE_CLOSE)
    {
        return;
    }
    
	uint32_t event = TX_MSG_EVENT_TICK;
	xQueueSend(gQueTxMsg, (void *)(&event), 0);
#endif
}

static void startSendMsg(int canIdx)
{
    volatile TxTargets_t *pTargets = &(gTxTargets[canIdx]);

    xSemaphoreTake(gTxTargets[canIdx].sem, portMAX_DELAY);

    if (pTargets->num == 0 /*|| isSilent()*/ || isBusoff(canIdx))
    {
        pTargets->num = 0;
        pTargets->idx = 0;
        pTargets->start = 0;
        pTargets->sendCnt = 0;

        xSemaphoreGive(gTxTargets[canIdx].sem);
        
        return;
    }

    //启动发送
    if (pTargets->start == 0)
    {
        //命令模式
        if (isCfgMode())
        {
            pTargets->num = 0;
            pTargets->idx = 0;
            pTargets->start = 0;
            pTargets->sendCnt = 0;
        }
        else
        {
            int status = arc_lock_save();

            pTargets->start = 1;

            if (pTargets->now == 0)
            {
                can_send_async(canIdx, 0);
            }

            arc_unlock_restore(status);
        }
    }
    else if (pTargets->idx < pTargets->num)
    {
        uint16_t idx = pTargets->idx;
        int cnt = 0, timeout = MS_TO_RTC(2);
        while (idx == pTargets->idx && cnt++ < timeout);

        int status = arc_lock_save();
        
        if (cnt >= timeout && pTargets->start)
        {
            can_send_async(canIdx, pTargets->idx);
        }

        arc_unlock_restore(status);
    }
    else
    {
        pTargets->num = 0;
        pTargets->idx = 0;
        pTargets->start = 0;
        pTargets->sendCnt = 0;
    }

    xSemaphoreGive(gTxTargets[canIdx].sem);
}

void canStartSend(void)
{
    uint32_t event = TX_MSG_EVENT_START;
    xQueueSend(gQueTxMsg, (void *)(&event), 0);  //--- 这个使用的是50ms触发
}

static int canSendMsg(int canCh, void *txBufParam, UINT32 ID)
{
    if (gTxTargets[canCh].num >= CAN_TX_BUF_SIZE)
    {
        return -1;
    }

    bool lock = false;
    int status = arc_lock_save();

    //信号量已被占用说明有其他线程使用，需要等待
    if (uxSemaphoreGetCount(gTxTargets[canCh].sem) == 0)
    {
        arc_unlock_restore(status);

        xSemaphoreTake(gTxTargets[canCh].sem, portMAX_DELAY);

        lock = true;

        status = arc_lock_save();
    }

    stCanTxMsg *pMsg = (stCanTxMsg *)&gTxTargets[canCh].msgBuf[gTxTargets[canCh].num];

    pMsg->id = ID;
    *(uint64_t *)pMsg->data = *(uint64_t *)txBufParam;

    gTxTargets[canCh].num++;

    arc_unlock_restore(status);

    if (lock)
    {
        xSemaphoreGive(gTxTargets[canCh].sem);
    }

    return E_OK;
}

static int can0_send(void *txBufParam, UINT32 ID)
{
    return canSendMsg(CAN_0_ID, txBufParam, ID);
}

static int can1_send(void *txBufParam, UINT32 ID)
{
    return canSendMsg(CAN_1_ID, txBufParam, ID);
}

static void send_frame_now(uint8_t canIdx ,void *txBufParam, UINT32 ID, uint8_t len)
{
    int j = 0;

    while (gTxTargets[canIdx].now != 0)
    {
        //超时直接丢弃buf数据
        if (j++ > CAN_DELAYTIME * 20 * 10)
        {
            EMBARC_PRINTF("send now out of time\r\n");
            gTxTargets[canIdx].now = 0;  
            break;
        }
    }

    int status = arc_lock_save();

	if (gTxTargets[canIdx].start == 1)
	{
        gTxTargets[canIdx].msgNow.id = ID;
        *(uint64_t *)gTxTargets[canIdx].msgNow.data = *(uint64_t *)txBufParam;
        gTxTargets[canIdx].msgNow.dlc = len;
        gTxTargets[canIdx].now = 1;
    }
    else
    {   
        gTxTargets[canIdx].now = 2;
        can_send_data(canIdx, ID, txBufParam, len);
    }

    arc_unlock_restore(status);
}

static void can0_send_frame_now(void *txBufParam, UINT32 ID, uint8_t len)
{
	send_frame_now(CAN_0_ID, txBufParam, ID, len);
}

static void can1_send_frame_now(void *txBufParam, UINT32 ID, uint8_t len)
{
	send_frame_now(CAN_1_ID, txBufParam, ID, len);
}

int getCanTxQue(int canCh, TickType_t waitTime, stCanTxMsg **pMsg)
{
    if (canCh > CAN_1_ID || pMsg == NULL)
    {
        return E_PAR;
    }

    if (xSemaphoreTake(gTxTargets[canCh].sem, waitTime) != pdTRUE)
    {
        return E_TMOUT;
    }

    *pMsg = (stCanTxMsg *)&gTxTargets[canCh].msgBuf[gTxTargets[canCh].num];

    return E_OK;
}

int putCanTxQue(int canCh, int msgNum)
{
    if (canCh > CAN_1_ID || gTxTargets[canCh].num + msgNum > CAN_TX_BUF_SIZE)
    {
        return E_PAR;
    }

    if (uxSemaphoreGetCount(gTxTargets[canCh].sem))
    {
        return E_OBJ;
    }

    gTxTargets[canCh].num += msgNum;
    
    xSemaphoreGive(gTxTargets[canCh].sem);

    return E_OK;
}

const TCANCtrl gCAN[2] =
{
    {
    		can0_send,
			can0_send_frame_now
    },
    {
    		can1_send,
			can1_send_frame_now
    }
};
//
//static void sendStatusNow(int canidx)
//{
//    char buf[8] = {0, };
//    uint32_t dtcFailMask = getDtcFailMask();
//    if (dtcFailMask)
//    {
//        buf[0] = (dtcFailMask & (1 << 10)) ? 0x1 : 0x3;
//    }
//    else
//    {
//        buf[0] = 0;
//    }
//
//    //gCAN[canidx].sendFrameNow(buf, RADAR_STATUS, 8);
//}

//busoff后立即复位，然后保持到200ms再开始发送报文，如果依然存在busoff则计数加一，并重新开始计算时间
static void can_busoff_process(uint8_t can_ch, int *busoff_cnt, volatile int8_t *busoff_flag,
							   int *busoff_timer, int *busoff_recover_timer)
{
    int busoff_check_time = (BUSOFF_QUICK_TIME/BUSOFF_TEST_PERIOD); //快恢复时间为 10*5 = 50ms  ， 慢恢复时间为 40*5 = 200ms
    if(((*busoff_cnt) >= BUSOFF_QUICK_CNT) && (BUSOFF_ALL_CNT>BUSOFF_QUICK_CNT))
    {
        busoff_check_time = (BUSOFF_SLOW_TIME/BUSOFF_TEST_PERIOD);
    }
    (*busoff_timer)++;

    if (*busoff_flag)
    {
        if (!gCanBusOffHold[can_ch] || *busoff_timer >= busoff_check_time)
        {
            *busoff_timer = 1;
            gCanBusOffHold[can_ch] = true;

            canReInit(can_ch);

            *busoff_flag = 0;
            if (++(*busoff_cnt) > BUSOFF_ALL_CNT)
            {
                *busoff_cnt = BUSOFF_ALL_CNT;
            }
            
            //EMBARC_PRINTF("reinit 1 time %d, *busoff_cnt %d\r\n", (int)(timer_rtc_count() / MS_TO_RTC(1)), *busoff_cnt);
        }

        *busoff_recover_timer = 0;
    }
    else if (*busoff_cnt == 0)
    {
        *busoff_timer = 0;
    }
    else
    {
        if (gCanBusOffHold[can_ch])
        {
            if (*busoff_timer >= busoff_check_time)
            {
                *busoff_timer = 0;
                *busoff_recover_timer = 0;
                gCanBusOffHold[can_ch] = false;

                //EMBARC_PRINTF("startSendMsg time %d, *busoff_cnt %d\r\n", (int)(timer_rtc_count() / MS_TO_RTC(1)), *busoff_cnt);
//                sendStatusNow(can_ch);
            }
        }
        else
        {
            (*busoff_recover_timer)++;
            if (*busoff_recover_timer >= (busoff_check_time/2))     //一般的恢复时间内都没有在产生busoff，将清除标志
            {
                //EMBARC_PRINTF("recover time %d, *busoff_cnt %d\r\n", (int)(timer_rtc_count() / MS_TO_RTC(1)), *busoff_cnt);
                *busoff_cnt = 0;
                *busoff_timer = 0;
            }
        }
    }   
}

static void can0_busoff_timer_handler(xTimerHandle xTimer)
{
	can_busoff_process(CAN_0_ID, &can0_busoff_cnt, &can0_busoff_flag, &can0_busoff_timer, &can0_busoff_recover_timer);
	can_busoff_process(CAN_1_ID, &can1_busoff_cnt, &can1_busoff_flag, &can1_busoff_timer, &can1_busoff_recover_timer);
}


/**
 * @description: CAN发送任务
 * @param {void} *param
 * @return {*}
 */
static void canTxTask(void *params)
{
    uint32_t event = 0;

    while (1)
    {
        if (xQueueReceive(gQueTxMsg, &event, portMAX_DELAY) == pdTRUE)
        {          
			if (!agingInfoSendEn())
			{
                if (event == TX_MSG_EVENT_TICK) //定时器触发
                {
                    app_target_process();
                }
			}
            
            if (event == TX_MSG_EVENT_TICK || event == TX_MSG_EVENT_START)
            {
            	if (gCanTxEnMask & (1 << CAN_0_ID)) 
                {
            		startSendMsg(CAN_0_ID);
            	}

            	if (gCanTxEnMask & (1 << CAN_1_ID)) 
                {
            		startSendMsg(CAN_1_ID);
            	}
            }
			else if (event == TX_MSG_EVENT_RESET_0 && (gCanTxEnMask & (1 << CAN_0_ID)))
            {
                canReInit(CAN_0_ID);
            }
            else if (event == TX_MSG_EVENT_RESET_1 && (gCanTxEnMask & (1 << CAN_1_ID)))
            {
                canReInit(CAN_1_ID);
            }
        }
    }
}

int canTransInit(void)
{
    for(int i = 0; i < 2;i++)
	{
		gTxTargets[i].start = 0;
        gTxTargets[i].num = 0;
		gTxTargets[i].idx = 0;
		gTxTargets[i].sendCnt = 0;
        gTxTargets[i].now = 0;
        gTxTargets[i].sem = xSemaphoreCreateBinary();
        if (gTxTargets[i].sem == NULL)
        {
            EMBARC_PRINTF("create tx queue sem error\r\n");
            return E_SYS;
        }

        xSemaphoreGive(gTxTargets[i].sem);
	}

    if ((gTxTaskHandle = createTask(canTxTask,
							 "canTxTask",
							 MPU_STACK_DEFAULT,
							 (void *)1,
							 TSK_PRIOR_HI)) == NULL)
	{
		EMBARC_PRINTF("create canTxTask task error\r\n");
        return -1;
	}

    gQueTxMsg = xQueueCreate(5, sizeof(uint32_t));
    if (gQueTxMsg == NULL)
    {
        EMBARC_PRINTF("create gQueTxMsg queue error\r\n");
        return -1;
    }

    xTimerCan0Busoff = xTimerCreate("busoff", pdMS_TO_TICKS(BUSOFF_TEST_PERIOD), pdTRUE, (void *)0, can0_busoff_timer_handler);
    if (xTimerCan0Busoff != NULL)
    {
        xTimerStart(xTimerCan0Busoff , 0);
    }
    else
    {
        EMBARC_PRINTF("create busoff timer failed\r\n");
        return -2;
    }
    
    xTimerTrackListSend = xTimerCreate("canTxTick", pdMS_TO_TICKS(50), pdTRUE, (void *)0, vTimerCan0SendCallback);
	if (xTimerTrackListSend != NULL)
	{
		xTimerStart(xTimerTrackListSend, 0);
	}
    else
    {
        EMBARC_PRINTF("create canTxTick timer failed\r\n");
        return -3;
    }
    
    can_confirmation_register(CAN_0_ID, can0TxCallBack);
    can_confirmation_register(CAN_1_ID, can1TxCallBack);

    can_error_event_register(CAN_0_ID, can0ErrorCallback);
    can_error_event_register(CAN_1_ID, can1ErrorCallback);

    return 0;
}

void canRxMsgProc(can_data_message_t *pMsg)
{
    TCANRxBuf rxMsg;

    rxMsg.id = pMsg->frame_params->msg_id;
    rxMsg.dlc = pMsg->frame_params->len;
    memcpy(rxMsg.data, pMsg->data, pMsg->frame_params->len);

    if (pMsg->can_ch == PCAN && DUTGetTestState() == DUT_MODE_CLOSE)
    {
        //CT_CAN信息处理
        if (rxMsg.id == CMD_FRAME_ID)
        {
            recv_can_cmd(pMsg->can_ch, rxMsg.data, 8);
        }
        else if (rxMsg.id == CAN_DBG_ID)
        {
            can_cmd_headler_dbg_cmd(pMsg->can_ch, rxMsg.data, 8);
        }
        else //接收数据
        {
            procRxCanFrame(pMsg->can_ch, &rxMsg);
        }
    }

    if(pMsg->can_ch == VCAN)
    {
        if (DUTGetTestState() == DUT_MODE_CLOSE)
        {
            TCANRxBuf rxBYDBuf;
            rxBYDBuf.id = pMsg->frame_params->msg_id;
            rxBYDBuf.dlc = pMsg->frame_params->len;
            memcpy(rxBYDBuf.data, pMsg->data, pMsg->frame_params->len);
            procRxBYDCanFrame(pMsg->can_ch, &rxBYDBuf);
        }
    }

    if(DUTCAN == pMsg->can_ch)
    {
        //DUT
        DUTProcessCANRXFrame(pMsg->can_ch, &rxMsg);
		canStartSend();
    }
}

