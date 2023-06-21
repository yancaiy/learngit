
#include <math.h>
#include "app_protocol.h"
#include "app_can_msg.h"
#include "app_vehicle.h"
#include "cfg.h"
#include "status.h"
#include "system_misc.h"
#include "target_proc.h"
#include "app_adc_hil.h"

#define RAD2DEG     57.295779513082323
#define DEG2RAD     0.017453292519943295f


//Crc8_8H2F 多项式0x12F
const uint8_t Crc8_8H2Ftable[] = {
    0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
    0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
    0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
    0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
    0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
    0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
    0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
    0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
    0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
    0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
    0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
    0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
    0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
    0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
    0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
    0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42,
};

uint8_t Crc8_8H2F(uint8_t *pData, uint8_t len)
{
	uint8_t crc8 = 0xFF;

	for (int8_t i = len - 1; i >= 0; i--)
	{
		crc8 ^= pData[i];
		crc8 = Crc8_8H2Ftable[crc8];
	}

	crc8 ^= 0xFF;
	return crc8;
}

void procRxCanFrame(uint8_t canIdx, TCANRxBuf *pBuf)
{
    static int tProVersion = PROTOCOL_VER_0;
    static int objNum = 0;

    /* --------------------------- for test --------------------------- */
//    EMBARC_PRINTF("--------- CAN id:0x%x, data:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", pBuf->id, pBuf->data[0],
//        pBuf->data[1], pBuf->data[2], pBuf->data[3], pBuf->data[4], pBuf->data[5], pBuf->data[6], pBuf->data[7]);

    //这里如果进行转换，只能进行bit的结构体操作，否则需要再将数据转换一次
    convert(pBuf->data);

    //复位
    if (pBuf->id == SET_RADAR_CFG && (*(uint64_t *)(pBuf->data) == CMD_BOOT_MAGIC))
    {
        system_reset();
    }
    //切换正常/调试模式 0x30n
    else if (pBuf->id == SET_RADAR_MODE)
    {
        char buf[8];
        stRadarModeMsg *rec = (stRadarModeMsg *)pBuf->data;
        if (*(uint64_t *)(pBuf->data) == CMD_READ_CFG)
        {
            stRadarModeMsg *pAck = (stRadarModeMsg *)buf;
            pAck->password = 0xFFFFFF;
            pAck->radar_mode = radar_config_using->debugModeSW;
            pAck->resv = 0;
            //发送 回读信息
            gCAN[canIdx].sendFrame(pAck, SET_RADAR_MODE);
        }
        else if (rec->password == PASSWORD_RADAR_MODE_SW)
        {
            setCfgDebugMode(rec->radar_mode);
            setCanTxEnMask(0x3); //CAN Tx都使能
        }
    }
    //3F0 - 大端，车速、曲率等输入
    else if (pBuf->id == SET_CAR_INFO)
    {
        stInCarInfoMsg *carInfo = (stInCarInfoMsg *)(pBuf->data);
        //车速
        if (carInfo->velocityValid)
        {
            VehicleSetSpeed((float)(carInfo->velocity / 100.0f), 0, 1);
        }

        //转向信号, 转向灯信号, 当微信号, 曲率
        if (carInfo->curvatureValid)
        {
            float Yawrate = (float)(((int16_t)carInfo->curvature) / 100.0f);
            VehicleSetYawRate(Yawrate, 1);

        	//有输入配置YawRate初始化是有效的
        	setCyroCaliFlag(1);
        }
    }
    //3F5 - 小端，本车速度输入
    else if (pBuf->id == SET_CAR_VEL_INTEL)
    {
        convert(pBuf->data);
        VehicleSetSpeed(((pBuf->data[1] << 8 | pBuf->data[0]) & 0x7FFF) * 0.01, ((pBuf->data[3] >> 5) & 0x01), 1); //bit29 dir
        convert(pBuf->data);
    }
    //3F6 - 小端，曲率输入
    else if (pBuf->id == SET_CAR_YAWRATE_INTEL)
    {
        convert(pBuf->data);
        float Yawrate = (float)((int16_t)(pBuf->data[3] << 8 | pBuf->data[2]) / 100.0f);
        Yawrate = (fabsf(Yawrate) > 0.5 ? Yawrate : 0);
        VehicleSetYawRate(Yawrate, 1);
    	//有输入配置YawRate初始化是有效的
    	setCyroCaliFlag(1);
        convert(pBuf->data);
    }
    else if (pBuf->id == SET_TIMESTAMP)
    {
        setTimestamp(*(uint64_t *)pBuf->data);
    }
#if HIL_FUNC
    else if (pBuf->id == SET_HIL_FUNC_FRAME_CNT)
    {
        if (ADC_DUMP_MAGIC_NUM == ((pBuf->data[7] << 24) | (pBuf->data[6] << 16) | (pBuf->data[5] << 8) | pBuf ->data[4]))
        {
			appAdcOrHilFuncParamSetupProc(ADC_HIL_FUNC_HIL, ((pBuf->data[2] << 16) | (pBuf->data[1] << 8) | pBuf ->data[0]), pBuf->data[3]);
		}
        else
        {
            EMBARC_PRINTF("HIL Func frame cnt set error: magic num check error\n");
        }
    }	
	else if (pBuf->id == SET_HIL_FUNC_RADAR_REBOOT)
	{
		EMBARC_PRINTF("Rebooting...\n");

		system_reset();
	}
	else if (pBuf->id == SET_HIL_FUNC_FRAME_INTERVAL)
	{
		appHilFuncFrameIntervalSet(((pBuf->data[3] << 24) | (pBuf->data[2] << 16) | (pBuf->data[1] << 8) | pBuf ->data[0]));
	}
#endif /* HIL_FUNC */

    convert(pBuf->data);
}

void procRxBYDCanFrame(uint8_t canIdx, TCANRxBuf *pBuf)
{
    switch(pBuf->id)
    {
        case 0x3F0:     //承泰协议车身信息
        {
            convert(pBuf->data);
            stInCarInfoMsg *carInfo = (stInCarInfoMsg *)(pBuf->data);
            //车速
            if (carInfo->velocityValid)
            {              
                VehicleSetSpeed((float)(carInfo->velocity / 100.0f), 0, 1);
                gHILPara[0] = carInfo->velocity / 360.0f;
			}

            //转向信号, 转向灯信号, 当微信号, 曲率
            if (carInfo->curvatureValid)
            {
                float Yawrate = (float)(((int16_t)carInfo->curvature) / 100.0f);

                //做一次中值滤波
                gHILPara[2] = Yawrate;
                Yawrate = doYawRateMedianFilter(Yawrate);
                VehicleSetYawRate(-Yawrate, 1);
                setCyroCaliFlag(1);
            }

            break;
        }
        default:
            break;
    }
}

void appProsendTargetsArs410(uint8_t canIdx, objectInfo_t *pOutObject, int targetNum)
{
    stCanTxMsg *pMsg = NULL;

	if (getCanTxQue(canIdx, portMAX_DELAY, &pMsg) != E_OK)
	{
		EMBARC_PRINTF("[Err] getCanTxQue() failed\r\n");
		return;
	}

	int32_t Idx = 0;
    int32_t i = 0;
    static uint8_t AliveCounter = 0;
    uint16_t FrsStatueMsgId  = 0x80;
    uint16_t FrsPart1StartId = 0x50;
    uint16_t FrsPart2StartId = 0x20;
    FRS_Statue   *statuObjMsg;
    FRS_ObjPart1 *ObjPart1;
    FRS_ObjPart2 *ObjPart2;
    float x, y;
    float YawRate, Speed;

    for(i = 0; i < targetNum; i++)
    {
        // FRS_ObjPart2 0x20
        ObjPart2 = (FRS_ObjPart2 *)pMsg[Idx].data;
        pMsg[Idx].id = FrsPart2StartId + i;
        pMsg[Idx].dlc = eDATA_LEN_8;
        // FRS_ObjPart1 0x50
        ObjPart1 = (FRS_ObjPart1 *)pMsg[Idx + targetNum].data;
        pMsg[Idx + targetNum].id = FrsPart1StartId + i;
        pMsg[Idx + targetNum].dlc = eDATA_LEN_8;

        x = pOutObject[i].range * cosf(pOutObject[i].heighAngle * DEG2RAD) * sinf(pOutObject[i].angle * DEG2RAD);
        y = pOutObject[i].range * cosf(pOutObject[i].heighAngle * DEG2RAD) * cosf(pOutObject[i].angle * DEG2RAD);

        ObjPart2->FRS_P2_00_Msg_AliveCounter     = AliveCounter & 0x0F;
        ObjPart2->FRS_P2_00_Obj_MeasFlag         = 0;   // For test.
        ObjPart2->FRS_P2_00_Obj_YVelRel          = (uint16_t)((-pOutObject[i].velocity + 102.4) / 0.1);
        ObjPart2->FRS_P2_00_Obj_Type             = 0;   // For test.
        ObjPart2->FRS_P2_00_Obj_XPos             = (uint16_t)((y +  0) / 0.015625);
        ObjPart2->FRS_P2_00_Obj_YPos             = (uint16_t)(((-x) + 64) / 0.015625);
        ObjPart2->FRS_P2_00_Obj_XVelRel          = (uint16_t)(((0) + 102.4) / 0.1);     // For test.
        ObjPart2->FRS_P2_00_Msg_CheckSum         = Crc8_8H2F((uint8_t *)ObjPart2 + 1, 7);
        
        ObjPart1->FRS_P1_00_Msg_AliveCounter   = AliveCounter & 0x0F;       // 每一次雷达传感器测量周期都要增加活计数器。
        ObjPart1->FRS_P1_00_Obj_ExstProb       = (uint16_t)((float)(75) / 1.5873);      // For test.
        ObjPart1->FRS_P1_00_Obj_XVelRel_Stdev  = 0x7F;  // For test.
        ObjPart1->FRS_P1_00_Obj_XAccRel        = (uint16_t)(((0) + 9.6) / 0.15);        // For test.
        ObjPart1->FRS_P1_00_Obj_ObstacleProb   = 0x0;   // For test. 对象成为障碍物的估计概率
        ObjPart1->FRS_P1_00_Obj_MotionPattern  = 0;     // For test.
        ObjPart1->FRS_P1_00_Obj_YPos_Stdev     = 0x7F;  // For test.
        ObjPart1->FRS_P1_00_Obj_ValidFlag      = 0x1;
        ObjPart1->FRS_P1_00_Obj_XPos_Stdev     = 0x7F;  // For test. 估计物体纵向位置的 3-sigma 标准差值
        ObjPart1->FRS_P1_00_Obj_UpdateFlag     = 0;     // For test. 是否是上一帧CAN已经发送的对象
        ObjPart1->FRS_P1_00_Obj_ID             = i & 0xFF;
        ObjPart1->FRS_P1_00_Msg_CheckSum       = Crc8_8H2F((uint8_t *)ObjPart1 + 1, 7);

        Idx++;
    }

    //FRS_Statue 0x80
    statuObjMsg = (FRS_Statue *)pMsg[Idx + targetNum].data;
    pMsg[Idx + targetNum].id = FrsStatueMsgId;
    pMsg[Idx + targetNum].dlc = eDATA_LEN_8;

    updateTimestamp();
    YawRate = VehicleGetYawRate();
    Speed = VehicleGetSpeed();

    statuObjMsg->FRS_Status_MisAlign   = 0x4;  // For test. 0x00:unknown 0x01:calibrated, 0x02:sensor mis-alignment detected, 0x03:calibration in progress, 0x04:uncalibratable, 0x05-07:reserved
    statuObjMsg->FRS_Status_HWErr      = 0;    // 0x0:HW is NOT failed  0x1:HW is failed
    statuObjMsg->FRS_Status_BlkProg    = 0x0;  // For test. 0x0:FRS is NOT blocked  0x1:FRS is blocked
    statuObjMsg->FRS_Fail              = 0x1;  // 0x0 = FRS radar is NOT working, 0x1 = FRS radar is working
    statuObjMsg->FRS_MeasEnabled       = 0;    // For test. 暂时用于放置隧道模式
    statuObjMsg->FRS_Host_Yaw          = fabsf(YawRate) > 102.4 ? (YawRate>0 ? 0x7FF : 0) :(uint16_t)((YawRate + 102.4) / 0.1);//左转弯定义为FRS_Host_Yaw正方向
    statuObjMsg->FRS_HostSpeed         = Speed>82.375 ? 0xFFF : (uint16_t)((Speed + 20) / 0.025);
    statuObjMsg->FRS_TimeStamp         = getTimestamp() / 1000 & 0xFFFF;
    statuObjMsg->FRS_Latency           = 100 / 2.5 / 2;     // For test. TO DO 暂时使用帧间隔时间/2.5
    statuObjMsg->FRS_Msg_AliveCounter  = AliveCounter & 0x0F;
    statuObjMsg->FRS_Msg_CheckSum      = Crc8_8H2F((uint8_t *)statuObjMsg + 1, 7);

    AliveCounter++;
    putCanTxQue(canIdx, 2 * targetNum + 1);
}

