/*
 * APP_DUT.c
 *
 *  Created on: 2018��5��7��
 *      Author: RonLiu
 */
#include <stdlib.h>
#include "sharedVar.h"
#include "system_misc.h"
#include "cfg.h"
#include "system_misc.h"
#include "can_trans.h"
//#include "app_uart_protoco.h"
#include "app_dut.h"
#include "sharedDef.h"
#include "typedefs.h"
#include "radio_ctrl.h"
#include "baseband_alps_FM_reg.h"
#include "arc_exception.h"
#include "baseband.h"
//#include "includes_app_h.h"
#include "flash_mmap.h"
#include "angleMapping.h"
//#include "radarSdsp.h"
//#include "app_eol_dbf.h"
//#include "target_proc.h"
//#include "pmic_max.h"

#define BB_WRITE_REG(bb_hw, RN, val) baseband_write_reg(bb_hw, BB_REG_##RN, val)

extern void reReadFlashParams(void);


// CAN RX MESSAGE ID
#define FACTORY_CAN_SET_WORK_REQ 0x100
#define FACTORY_CAN_SET_WAVE_REQ 0x101
#define FACTORY_CAN_SELFTEST_REQ 0x102
#define FACTORY_CAN_OEM_VERSION_REQ 0x103
#define FACTORY_CAN_OEM_NAME_REQ 0x104
#define FACTORY_CAN_OEM_SN_REQ 0x105
#define FACTORY_CAN_OEM_PCB_SN_REQ 0x106
#define FACTORY_CAN_OEM_ETHERNET_REQ 0x107
#define FACTORY_CAN_OEM_RADAR_ID_REQ 0x108
#define FACTORY_CAN_SET_ANGLE_CAL_MOLE_REQ 0x109    //设置雷达计算角度的校准模式
#define FACTORY_CAN_WRITE_RANGE_CAL_REQ 0x110
#define FACTORY_CAN_WRITE_VEL_CAL_REQ 0x111
#define FACTORY_CAN_WRITE_ANGLE_CAL_REQ 0x112
#define FACTORY_CAN_R_PA_CAL_REQ 0x113
#define FACTORY_CAN_W_PA_CAL_REQ 0x114
#define FACTORY_CAN_WRITE_INFO_REQ 0x115
#define FACTORY_BURN_CFG_REQ 0x116
#define FACTORY_ERASE_RADAR_CFG 0x117
#define FACTORY_REREAD_FLASH_CFG 0x118 //重新读取Flash的值
#define FACTORY_CAN_RX_POWER_REQ 0x120
#define FACTORY_CAN_WR_ANG_MAP_REQ				0x121
#define FACTORY_CAN_EN_ANG_MAP_REQ				0x122
#define FACTORY_CAN_READ_DTC_STATUE_REQ			0x123
#define FACTORY_CAN_NOISE_REQ 					0x124 //读取底噪	
#define FACTORY_CAN_W_RCS_CAL_REQ 				0x127 //设置RCS校准值

#define FACTORY_CAN_W_DUT_TEST_CFG				0x130 //配置测试参数，使用跟踪前还是跟踪后的目标
#define FACTORY_CAN_ACC_SELF_CALC_CFG			0x131 //陀螺仪校准测试命令
#define FACTORY_CAN_WTD_TEST_CMD				0x136 //看门狗测试命令

#define FACTORY_CAN_W_CHANGE_OBJECT_LIST 		0x150 //切换profile
#define FACTORY_CAN_SEND_WAVE_REQ 				0x153 //电机转动角度1度时发出该信号，触发一次发波并上传跟踪列表以及2DFFT数据
#define FACTORY_DBF_WRITE_REQ                   0x154 //收到0x154后，执行dbf因子写入操作
#define FACTORY_SM_TEST_REQ 					0x155 //SM测试

// CAN TX MESSAGE ID
#define FACTORY_CAN_SET_WORK_ANS 0x200
#define FACTORY_CAN_SET_WAVE_ANS 0x201
#define FACTORY_CAN_SELFTEST_ANS 0x202
#define FACTORY_CAN_OEM_VERSION_ANS 0x203
#define FACTORY_CAN_OEM_NAME_ANS 0x204
#define FACTORY_CAN_OEM_SN_ANS 0x205
#define FACTORY_CAN_OEM_PCB_SN_ANS 0x206
#define FACTORY_CAN_OEM_ETHERNET_ANS 0x207
#define FACTORY_CAN_OEM_RADAR_ID_ANS 0x208
#define FACTORY_CAN_ANGLE_CAL_MOLE_ANS 0x209    //设置雷达计算角度的校准模式响应
#define FACTORY_CAN_WRITE_RANGE_CAL_ANS 0x210
#define FACTORY_CAN_WRITE_VEL_CAL_ANS 0x211
#define FACTORY_CAN_WRITE_ANGLE_CAL_ANS 0x212
#define FACTORY_CAN_R_PA_CAL_ANS 0x213
#define FACTORY_CAN_W_PA_CAL_ANS 0x214
#define FACTORY_CAN_WRITE_INFO_ANS 0x215
#define FACTORY_BURN_CFG_ANS 0x216
#define FACTORY_ERASE_RADAR_CFG_ANS 0x217
#define FACTORY_REREAD_FLASH_CFG_ANS 0x218
#define FACTORY_CAN_RX_POWER_ANS 0x220
#define FACTORY_CAN_WR_ANG_MAP_ANS				0x221
#define FACTORY_CAN_EN_ANG_MAP_ANS				0x222
#define FACTORY_CAN_READ_DTC_STATUE_ANS			0x223
#define FACTORY_CAN_NOISE_ANS 					0x224
#define FACTORY_CAN_SET_RCS_CAL_ANS 			0x227  //应答RCS的标定值命令

#define FACTORY_CAN_W_DUT_TEST_CFG_ANS			0x230
#define FACTORY_CAN_ACC_SELF_CALC_CFG_ANS		0x231 //陀螺仪校准测试命令响应
#define FACTORY_CAN_WTD_TEST_CMD_ANS			0x236 //看门狗测试命令回应
#define FACTORY_CAN_R_CHANGE_OBJECT_LIST  		0x250 //切换profile回复
#define FACTORY_CAN_SEND_WAVE_ANS 				0x253 //一度一校准触发模式的应答
#define FACTORY_SM_TEST_ANS 					0x255 //SM测试回复

/*  */
#define FACTORY_WR_WRITE 0x0
#define FACTORY_WR_READ 0x1
#define FACTORY_RESULT_ERROR 0x0
#define FACTORY_RESULT_SUCCESS 0x1
#define FACTORY_VERSION_HW 0x1
#define FACTORY_VERSION_SW 0x2
#define FACTORY_VERSION_CALI 0x3
#define FACTORY_UDS_VERSION_HW 0x4
#define FACTORY_UDS_VERSION_SW 0x5
#define FACTORY_UDS_VERSION_BOOT 0x6


#define FACTORY_ETHERNET_MAC 0x1
#define FACTORY_ETHERNET_IP 0x2
#define FACTORY_CALI_PHASE 0x1
#define FACTORY_CALI_DISTANCE 0x2
#define FACTORY_PCB_SN_BB 0x0
#define FACTORY_PCB_SN_RF 0x1
#define FACTORY_PA_CPLX_RE_INT 0x0
#define FACTORY_PA_CPLX_IM_INT 0x1
#define FACTORY_PA_CPLX_RE_FLT 0x2
#define FACTORY_PA_CPLX_IM_FLT 0x3

#define DUT_CAL_MODE_PROFILE_NUM 3 //220plus

/* set work */
typedef union tagDUTSetWorkReqMsg {
    UINT64 word;
    struct
    {
        UINT64 SW_Mode :8;
        UINT64 SW_PassWord 	:24;
        UINT64 SW_RandNum 	:24;
		UINT64 SW_CheckSum 	:8;	
    } bit_big;
    struct
    {
		UINT64 SW_CheckSum 	:8;	    
        UINT64 SW_RandNum 	:24;
        UINT64 SW_PassWord 	:24;
        UINT64 SW_Mode :8;
    } bit;
} TDUTSetWorkReqMsg;

typedef union tagDUTSetWorkAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 SW_Status : 8;
        UINT64 SW_Password : 24;
		UINT8  SW_ProfileAmount : 8;
		UINT8  SW_ChannelAmount : 8;
		UINT16 SW_Ans_Rev : 16;
    } bit_big;
    struct
    {
		UINT16 SW_Ans_Rev : 16;		
		UINT8  SW_ChannelAmount : 8;		
		UINT8  SW_ProfileAmount : 8;
        UINT64 SW_Password : 24;
        UINT64 SW_Status : 8;
    } bit;

} TDUTSetWorkAnsMsg;

typedef union tagDUTSetWorkSecureAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 SW_Status 	:8;
        UINT64 SW_PassWord 	:24;
		UINT64 SW_RandNum 	:24;
		UINT64 SW_ChechSum 	:8;		
    } bit_big;
    struct
    {
		UINT64 SW_ChechSum 	:8;		
		UINT64 SW_RandNum 	:24;
        UINT64 SW_PassWord 	:24;
        UINT64 SW_Status 	:8;
    } bit;

} TDUTSetWorkSecureAnsMsg;

/* set wave */
typedef union tagDUTSetWaveReqMsg {
    UINT64 word;
    struct
    {
        UINT64 WV_Mode : 3;
        UINT64 WV_Profile : 5;
        UINT64 WV_CW_Fc1 : 7;
        UINT64 WV_CW_Fc2 : 10;
        UINT64 WV_BW : 9;
        UINT64 WV_TX_GAIN : 8;
        UINT64 WV_Req_Rev : 22;
    } bit_big;
    struct
    {
        UINT64 WV_Req_Rev : 22;
        UINT64 WV_TX_GAIN : 8;
        UINT64 WV_BW : 9;
        UINT64 WV_CW_Fc2 : 10;
        UINT64 WV_CW_Fc1 : 7;
        UINT64 WV_Profile : 5;
        UINT64 WV_Mode : 3;
    } bit;
} TDUTSetWaveReqMsg;

typedef union tagDUTSetWaveAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 WV_Status : 3;
        UINT64 WV_Profile : 5;
        UINT64 WV_CW_Fc1 : 7;
        UINT64 WV_CW_Fc2 : 10;
        UINT64 WV_BW : 9;
        UINT64 WV_TX_GAIN : 8;
        UINT64 WV_Ans_Rev : 22;
    } bit_big;
    struct
    {
        UINT64 WV_Ans_Rev : 22;
        UINT64 WV_TX_GAIN : 8;
        UINT64 WV_BW : 9;
        UINT64 WV_CW_Fc2 : 10;
        UINT64 WV_CW_Fc1 : 7;
        UINT64 WV_Profile : 5;
        UINT64 WV_Status : 3;
    } bit;
} TDUTSetWaveAnsMsg;

/* self test */
typedef union tagDUTSelfTestReqMsg {
    UINT64 word;
    struct
    {
        UINT64 CMD_Mode : 1;
        UINT64 ST_Req_Rev0 : 7;
        UINT64 ST_Req_Rev1 : 8;
        UINT64 ST_Req_Rev2 : 8;
        UINT64 ST_Req_Rev3 : 8;
        UINT64 ST_Req_Rev4 : 8;
        UINT64 ST_Req_Rev5 : 8;
        UINT64 ST_Req_Rev6 : 8;
        UINT64 ST_Req_Rev7 : 8;
    } bit_big;
    struct
    {
        UINT64 ST_Req_Rev7 : 8;
        UINT64 ST_Req_Rev6 : 8;
        UINT64 ST_Req_Rev5 : 8;
        UINT64 ST_Req_Rev4 : 8;
        UINT64 ST_Req_Rev3 : 8;
        UINT64 ST_Req_Rev2 : 8;
        UINT64 ST_Req_Rev1 : 8;
        UINT64 ST_Req_Rev0 : 7;
        UINT64 CMD_Mode : 1;
    } bit;
} TDUTSelfTestReqMsg;

typedef union tagDUTSelfTestAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 ST_TEMP : 7;
        UINT64 ST_MCU_ERR : 1;
        UINT64 ST_MMIC_ERR : 1;
        UINT64 ST_NVM_ERR : 1;
        UINT64 ST_PMIC_ERR : 1;
        UINT64 ST_Ans_Rev1 : 5;
        UINT64 ST_Ans_Rev2 : 8;
        UINT64 ST_Ans_Rev3 : 8;
        UINT64 ST_Ans_Rev4 : 8;
        UINT64 ST_Ans_Rev5 : 8;
        UINT64 ST_Ans_Rev6 : 8;
        UINT64 ST_Ans_Rev7 : 8;
    } bit_big;
    struct
    {
        UINT64 ST_Ans_Rev7 : 8;
        UINT64 ST_Ans_Rev6 : 8;
        UINT64 ST_Ans_Rev5 : 8;
        UINT64 ST_Ans_Rev4 : 8;
        UINT64 ST_Ans_Rev3 : 8;
        UINT64 ST_Ans_Rev2 : 8;
        UINT64 ST_Ans_Rev1 : 5;
        UINT64 ST_PMIC_ERR : 1;
        UINT64 ST_NVM_ERR : 1;
        UINT64 ST_MMIC_ERR : 1;
        UINT64 ST_MCU_ERR : 1;
        UINT64 ST_TEMP : 7;
    } bit;
} TDUTSelfTestAnsMsg;

/* OEM version */
typedef union tagDUTVersionReqMsg {
    UINT64 word;
    struct
    {
        UINT64 VER_WR : 1;   /* 0: Single Write 1: Single Read */
        UINT64 VER_Type : 7; /* 0:NUL 1:HW_VER 2:SW_VER 3:CALI_VER .... */
        UINT64 VER_Byte0 : 8;
        UINT64 VER_Byte1 : 8;
        UINT64 VER_Byte2 : 8;
        UINT64 VER_Byte3 : 8;
        UINT64 VER_Byte4 : 8;
        UINT64 VER_Byte5 : 8;
        UINT64 VER_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 VER_Byte6 : 8;
        UINT64 VER_Byte5 : 8;
        UINT64 VER_Byte4 : 8;
        UINT64 VER_Byte3 : 8;
        UINT64 VER_Byte2 : 8;
        UINT64 VER_Byte1 : 8;
        UINT64 VER_Byte0 : 8;
        UINT64 VER_Type : 7; /* 0:NUL 1:HW_VER 2:SW_VER 3:CALI_VER .... */
        UINT64 VER_WR : 1;   /* 0: Single Write 1: Single Read */
    } bit;
} TDUTVersionReqMsg;

typedef union tagDUTVersionAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 VER_Result : 1; /* 0: ERROR 1: SUCCESS */
        UINT64 VER_Type : 7;   /* 0:NUL 1:HW_VER 2:SW_VER 3:CALI_VER .... */
        UINT64 VER_Byte0 : 8;
        UINT64 VER_Byte1 : 8;
        UINT64 VER_Byte2 : 8;
        UINT64 VER_Byte3 : 8;
        UINT64 VER_Byte4 : 8;
        UINT64 VER_Byte5 : 8;
        UINT64 VER_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 VER_Byte6 : 8;
        UINT64 VER_Byte5 : 8;
        UINT64 VER_Byte4 : 8;
        UINT64 VER_Byte3 : 8;
        UINT64 VER_Byte2 : 8;
        UINT64 VER_Byte1 : 8;
        UINT64 VER_Byte0 : 8;
        UINT64 VER_Type : 7;   /* 0:NUL 1:HW_VER 2:SW_VER 3:CALI_VER .... */
        UINT64 VER_Result : 1; /* 0: ERROR 1: SUCCESS */
    } bit;
} TDUTVersionAnsMsg;

/* OEM NAME */
typedef union tagDUTNAMEReqMsg {
    UINT64 word;
    struct
    {
        UINT64 NAME_WR : 1; /* 0: single Write, 1: Single Read */
        UINT64 NAME_Flag : 2;
        UINT64 NAME_Number : 5;
        UINT64 NAME_Byte0 : 8;
        UINT64 NAME_Byte1 : 8;
        UINT64 NAME_Byte2 : 8;
        UINT64 NAME_Byte3 : 8;
        UINT64 NAME_Byte4 : 8;
        UINT64 NAME_Byte5 : 8;
        UINT64 NAME_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 NAME_Byte6 : 8;
        UINT64 NAME_Byte5 : 8;
        UINT64 NAME_Byte4 : 8;
        UINT64 NAME_Byte3 : 8;
        UINT64 NAME_Byte2 : 8;
        UINT64 NAME_Byte1 : 8;
        UINT64 NAME_Byte0 : 8;
        UINT64 NAME_Number : 5;
        UINT64 NAME_Flag : 2;
        UINT64 NAME_WR : 1; /* 0: single Write, 1: Single Read */
    } bit;
} TDUTNAMEReqMsg;

typedef union tagDUTNAMEAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 NAME_Result : 1;
        UINT64 NAME_Flag : 2;
        UINT64 NAME_Number : 5;
        UINT64 NAME_Byte0 : 8;
        UINT64 NAME_Byte1 : 8;
        UINT64 NAME_Byte2 : 8;
        UINT64 NAME_Byte3 : 8;
        UINT64 NAME_Byte4 : 8;
        UINT64 NAME_Byte5 : 8;
        UINT64 NAME_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 NAME_Byte6 : 8;
        UINT64 NAME_Byte5 : 8;
        UINT64 NAME_Byte4 : 8;
        UINT64 NAME_Byte3 : 8;
        UINT64 NAME_Byte2 : 8;
        UINT64 NAME_Byte1 : 8;
        UINT64 NAME_Byte0 : 8;
        UINT64 NAME_Number : 5;
        UINT64 NAME_Flag : 2;
        UINT64 NAME_Result : 1;
    } bit;
} TDUTNAMEAnsMsg;

/* SN */
typedef union tagDUTSNReqMsg {
    UINT64 word;
    struct
    {
        UINT64 SN_WR : 1;
        UINT64 SN_Flag : 2;
        UINT64 SN_Number : 5;
        UINT64 SN_Byte0 : 8;
        UINT64 SN_Byte1 : 8;
        UINT64 SN_Byte2 : 8;
        UINT64 SN_Byte3 : 8;
        UINT64 SN_Byte4 : 8;
        UINT64 SN_Byte5 : 8;
        UINT64 SN_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 SN_Byte6 : 8;
        UINT64 SN_Byte5 : 8;
        UINT64 SN_Byte4 : 8;
        UINT64 SN_Byte3 : 8;
        UINT64 SN_Byte2 : 8;
        UINT64 SN_Byte1 : 8;
        UINT64 SN_Byte0 : 8;
        UINT64 SN_Number : 5;
        UINT64 SN_Flag : 2;
        UINT64 SN_WR : 1;
    } bit;
} TDUTSNReqMsg;

typedef union tagDUTSNAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 SN_Result : 1;
        UINT64 SN_Flag : 2;
        UINT64 SN_Number : 5;
        UINT64 SN_Byte0 : 8;
        UINT64 SN_Byte1 : 8;
        UINT64 SN_Byte2 : 8;
        UINT64 SN_Byte3 : 8;
        UINT64 SN_Byte4 : 8;
        UINT64 SN_Byte5 : 8;
        UINT64 SN_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 SN_Byte6 : 8;
        UINT64 SN_Byte5 : 8;
        UINT64 SN_Byte4 : 8;
        UINT64 SN_Byte3 : 8;
        UINT64 SN_Byte2 : 8;
        UINT64 SN_Byte1 : 8;
        UINT64 SN_Byte0 : 8;
        UINT64 SN_Number : 5;
        UINT64 SN_Flag : 2;
        UINT64 SN_Result : 1;
    } bit;
} TDUTSNAnsMsg;

/* PCB SN */
typedef union tagDUTPCBSNReqMsg {
    UINT64 word;
    struct
    {
        UINT64 PCB_SN_WR : 1;
        UINT64 PCB_SN_Flag : 2;
        UINT64 PCB_SN_Number : 5;
        UINT64 PCB_SN_Byte0 : 8;
        UINT64 PCB_SN_Byte1 : 8;
        UINT64 PCB_SN_Byte2 : 8;
        UINT64 PCB_SN_Byte3 : 8;
        UINT64 PCB_SN_Byte4 : 8;
        UINT64 PCB_SN_Byte5 : 8;
        UINT64 PCB_SN_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 PCB_SN_Byte6 : 8;
        UINT64 PCB_SN_Byte5 : 8;
        UINT64 PCB_SN_Byte4 : 8;
        UINT64 PCB_SN_Byte3 : 8;
        UINT64 PCB_SN_Byte2 : 8;
        UINT64 PCB_SN_Byte1 : 8;
        UINT64 PCB_SN_Byte0 : 8;
        UINT64 PCB_SN_Number : 5;
        UINT64 PCB_SN_Flag : 2;
        UINT64 PCB_SN_WR : 1;
    } bit;
} TDUTPCBSNReqMsg;

typedef union tagDUTPCBSNAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 PCB_SN_Result : 1;
        UINT64 PCB_SN_Flag : 2;
        UINT64 PCB_SN_Number : 5;
        UINT64 PCB_SN_Byte0 : 8;
        UINT64 PCB_SN_Byte1 : 8;
        UINT64 PCB_SN_Byte2 : 8;
        UINT64 PCB_SN_Byte3 : 8;
        UINT64 PCB_SN_Byte4 : 8;
        UINT64 PCB_SN_Byte5 : 8;
        UINT64 PCB_SN_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 PCB_SN_Byte6 : 8;
        UINT64 PCB_SN_Byte5 : 8;
        UINT64 PCB_SN_Byte4 : 8;
        UINT64 PCB_SN_Byte3 : 8;
        UINT64 PCB_SN_Byte2 : 8;
        UINT64 PCB_SN_Byte1 : 8;
        UINT64 PCB_SN_Byte0 : 8;
        UINT64 PCB_SN_Number : 5;
        UINT64 PCB_SN_Flag : 2;
        UINT64 PCB_SN_Result : 1;
    } bit;
} TDUTPCBSNAnsMsg;

/* SN ETHERNET */
typedef union tagDUTETHERNETReqMsg {
    UINT64 word;
    struct
    {
        UINT64 ETHERNET_WR : 1;   /* 0: single Write, 1: single Read */
        UINT64 ETHERNET_Type : 7; /* 0:NUL, 1:MAC, 2:IP,  */
        UINT64 ETHERNET_Byte0 : 8;
        UINT64 ETHERNET_Byte1 : 8;
        UINT64 ETHERNET_Byte2 : 8;
        UINT64 ETHERNET_Byte3 : 8;
        UINT64 ETHERNET_Byte4 : 8;
        UINT64 ETHERNET_Byte5 : 8;
        UINT64 ETHERNET_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 ETHERNET_Byte6 : 8;
        UINT64 ETHERNET_Byte5 : 8;
        UINT64 ETHERNET_Byte4 : 8;
        UINT64 ETHERNET_Byte3 : 8;
        UINT64 ETHERNET_Byte2 : 8;
        UINT64 ETHERNET_Byte1 : 8;
        UINT64 ETHERNET_Byte0 : 8;
        UINT64 ETHERNET_Type : 7; /* 0:NUL, 1:MAC, 2:IP,  */
        UINT64 ETHERNET_WR : 1;   /* 0: single Write, 1: single Read */
    } bit;
} TDUTETHERNETReqMsg;

typedef union tagDUTETHERNETAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 ETHERNET_Result : 1;
        UINT64 ETHERNET_Type : 7;
        UINT64 ETHERNET_Byte0 : 8;
        UINT64 ETHERNET_Byte1 : 8;
        UINT64 ETHERNET_Byte2 : 8;
        UINT64 ETHERNET_Byte3 : 8;
        UINT64 ETHERNET_Byte4 : 8;
        UINT64 ETHERNET_Byte5 : 8;
        UINT64 ETHERNET_Byte6 : 8;
    } bit_big;
    struct
    {
        UINT64 ETHERNET_Byte6 : 8;
        UINT64 ETHERNET_Byte5 : 8;
        UINT64 ETHERNET_Byte4 : 8;
        UINT64 ETHERNET_Byte3 : 8;
        UINT64 ETHERNET_Byte2 : 8;
        UINT64 ETHERNET_Byte1 : 8;
        UINT64 ETHERNET_Byte0 : 8;
        UINT64 ETHERNET_Type : 7;
        UINT64 ETHERNET_Result : 1;
    } bit;
} TDUTETHERNETAnsMsg;

/* radar id */
typedef union tagDUTRadarIdReqMsg {
    UINT64 word;
    struct
    {
        UINT64 RadarId_WR : 1; /* 1: single Write, 0: single Read */
        UINT64 RadarId : 7;
        UINT64 resv : 56;
    } bit_big;
    struct
    {
        UINT64 resv : 56;
        UINT64 RadarId : 7;
        UINT64 RadarId_WR : 1; /* 1: single Write, 0: single Read */
    } bit;
} DUTRadarIdReqMsg;

typedef union tagDUTRadarIdAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 RadarId_Result : 1;
        UINT64 RadarId : 7;
        UINT64 resv : 56;
    } bit_big;
    struct
    {
        UINT64 resv : 56;
        UINT64 RadarId : 7;
        UINT64 RadarId_Result : 1;
    } bit;
} DUTRadarIdAnsMsg;

/* cal range */
typedef union tagDUTCalRangeReqMsg {
    UINT64 word;
    struct
    {
        UINT64 Range_Ref_1 : 12;
        UINT64 Range_Tst_1 : 12;
        UINT64 Range_Rev_1 : 8;
        UINT64 Range_Ref_2 : 12;
        UINT64 Range_Tst_2 : 12;
        UINT64 Range_Rev_2 : 8;
    } bit_big;
    struct
    {
        UINT64 Range_Rev_2 : 8;
        UINT64 Range_Tst_2 : 12;
        UINT64 Range_Ref_2 : 12;
        UINT64 Range_Rev_1 : 8;
        UINT64 Range_Tst_1 : 12;
        UINT64 Range_Ref_1 : 12;
    } bit;
} TDUTCalRangeReqMsg;

//生产测试配置
typedef union tagDUTTestCfg{
	UINT64 word;
	struct {
		UINT64 DUTSendobjType		:4; //生产测试发送的目标类型：原始点还是跟踪后的点
		UINT64 agingTest            :4; //老化测试
		UINT64 protocolVersion      :8; //非工厂测试模式的 协议类型，用于不同精度发射 ,高四位为密钥
		UINT64 speedVagueCheckEn	:4; //速度解模糊回验功能配置
		UINT64 resv     			:44;
	}bit_big;
	struct {
		UINT64 resv     			:44;
		UINT64 speedVagueCheckEn	:4; //速度解模糊回验功能配置
		UINT64 protocolVersion      :8; //非工厂测试模式的 协议类型，用于不同精度发射 ,高四位为密钥
		UINT64 agingTest            :4; //老化测试
		UINT64 DUTSendobjType		:4; //生产测试发送的目标类型：原始点还是跟踪后的点
	} bit;
}TDUTTestCfgMsg;

//测试命令
typedef union tagDUTTestCmd{
	UINT64 word;
	struct {
		UINT64 PassWord		:32; //测试密钥
		UINT64 TestType     :8; //测试类型
		UINT64 TestAns     	:8; //测试应答、错误码等
		UINT64 resv     	:16; //保留
	}bit_big;
	struct {
		UINT64 resv     	:16; //保留
		UINT64 TestAns     	:8; //测试应答、错误码等
		UINT64 TestType     :8; //测试类型
		UINT64 PassWord		:32; //测试密钥
	}bit;
}TDUTTestCmdMsg;

typedef union tagDUTCalRangeAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 Range_Ref_1 : 12;
        UINT64 Range_Tst_1 : 12;
        UINT64 Range_Rev_1 : 8;
        UINT64 Range_Ref_2 : 12;
        UINT64 Range_Tst_2 : 12;
        UINT64 Range_Rev_2 : 8;
    } bit_big;
    struct
    {
        UINT64 Range_Rev_2 : 8;
        UINT64 Range_Tst_2 : 12;
        UINT64 Range_Ref_2 : 12;
        UINT64 Range_Rev_1 : 8;
        UINT64 Range_Tst_1 : 12;
        UINT64 Range_Ref_1 : 12;
    } bit;
} TDUTCalRangeAnsMsg;

/* cal vel */
typedef union tagDUTCalVelocityReqMsg {
    UINT64 word;
    struct
    {
        UINT64 Velocity_Ref_1 : 12;
        UINT64 Velocity_Tst_1 : 12;
        UINT64 Velocity_Rev_1 : 8;
        UINT64 Velocity_Ref_2 : 12;
        UINT64 Velocity_Tst_2 : 12;
        UINT64 Velocity_Rev_2 : 8;
    } bit_big;
    struct
    {
        UINT64 Velocity_Rev_2 : 8;
        UINT64 Velocity_Tst_2 : 12;
        UINT64 Velocity_Ref_2 : 12;
        UINT64 Velocity_Rev_1 : 8;
        UINT64 Velocity_Tst_1 : 12;
        UINT64 Velocity_Ref_1 : 12;
    } bit;
} TDUTCalVelocityReqMsg;

typedef union tagDUTCalVelocityAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 Velocity_Ref_1 : 12;
        UINT64 Velocity_Tst_1 : 12;
        UINT64 Velocity_Rev_1 : 8;
        UINT64 Velocity_Ref_2 : 12;
        UINT64 Velocity_Tst_2 : 12;
        UINT64 Velocity_Rev_2 : 8;
    } bit_big;
    struct
    {
        UINT64 Velocity_Rev_2 : 8;
        UINT64 Velocity_Tst_2 : 12;
        UINT64 Velocity_Ref_2 : 12;
        UINT64 Velocity_Rev_1 : 8;
        UINT64 Velocity_Tst_1 : 12;
        UINT64 Velocity_Ref_1 : 12;
    } bit;
} TDUTCalVelocityAnsMsg;

/* cal Az */
typedef union tagDUTCalAngleReqMsg {
    UINT64 word;
    struct
    {
        UINT64 Angle_Ref_1 : 11;
        UINT64 Angle_Tst_1 : 11;
        UINT64 Angle_Rev_1 : 10;
        UINT64 Angle_Ref_2 : 11;
        UINT64 Angle_Tst_2 : 11;
        UINT64 Angle_Rev_2 : 10;
    } bit_big;
    struct
    {
        UINT64 Angle_Rev_2 : 10;
        UINT64 Angle_Tst_2 : 11;
        UINT64 Angle_Ref_2 : 11;
        UINT64 Angle_Rev_1 : 10;
        UINT64 Angle_Tst_1 : 11;
        UINT64 Angle_Ref_1 : 11;
    } bit;
} TDUTCalAngleReqMsg;

typedef union tagDUTCalAngleAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 Angle_Ref_1 : 11;
        UINT64 Angle_Tst_1 : 11;
        UINT64 Angle_Rev_1 : 10;
        UINT64 Angle_Ref_2 : 11;
        UINT64 Angle_Tst_2 : 11;
        UINT64 Angle_Rev_2 : 10;
    } bit_big;
    struct
    {
        UINT64 Angle_Rev_2 : 10;
        UINT64 Angle_Tst_2 : 11;
        UINT64 Angle_Ref_2 : 11;
        UINT64 Angle_Rev_1 : 10;
        UINT64 Angle_Tst_1 : 11;
        UINT64 Angle_Ref_1 : 11;
    } bit;
} TDUTCalAngleAnsMsg;

/* Write PA CALI */
typedef union tagDUTWritePACaliReqMsg {
    UINT64 word;
    struct
    {
        UINT64 W_PA_Type : 4;
        UINT64 W_PA_Profile : 8;
        UINT64 W_PA_Channel : 8;
        UINT64 W_PA_Req_Resv : 12;
		UINT64 W_PA_Value : 32;    		
    } bit_big;
    struct
    {
        UINT64 W_PA_Value : 32;    
        UINT64 W_PA_Req_Resv : 12;
        UINT64 W_PA_Channel : 8;
        UINT64 W_PA_Profile : 8;
        UINT64 W_PA_Type : 4;
    } bit;
} TDUTWritePACaliReqMsg;

typedef union tagDUTWritePACaliAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 R_PA_Type : 4;
        UINT64 R_PA_Profile : 8;
        UINT64 R_PA_Channel : 8;
        UINT64 R_PA_Ans_Rev : 12;
		UINT64 R_PA_Value : 32;    	
    } bit_big;
    struct
    {
        UINT64 R_PA_Value : 32;    
        UINT64 R_PA_Ans_Rev : 12;
        UINT64 R_PA_Channel : 8;
        UINT64 R_PA_Profile : 8;
        UINT64 R_PA_Type : 4;
    } bit;
} TDUTWritePACaliAnsMsg;

/* RX Power */
typedef union tagDUTRXPowerReqMsg {
    UINT64 word;
    struct
    {
        UINT64 RX_Group : 4;
        UINT64 RX_Req_Rev0 : 4;
        UINT64 RX_Req_Rev1 : 8;
        UINT64 RX_Req_Rev2 : 16;
        UINT64 RX_Req_Rev3 : 16;
        UINT64 RX_Req_Rev4 : 16;
    } bit_big;
    struct
    {
        UINT64 RX_Req_Rev4 : 16;
        UINT64 RX_Req_Rev3 : 16;
        UINT64 RX_Req_Rev2 : 16;
        UINT64 RX_Req_Rev1 : 8;
        UINT64 RX_Req_Rev0 : 4;
        UINT64 RX_Group : 4;
    } bit;
} TDUTRXPowerReqMsg;

typedef union tagDUTRXPowerAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 RX_Group : 4;
        UINT64 RX_1_Power : 11;
        UINT64 RX_2_Power : 11;
        UINT64 RX_3_Power : 11;
        UINT64 RX_4_Power : 11;
        UINT64 RX_Ans_Rev : 16;
    } bit_big;
    struct
    {
        UINT64 RX_Ans_Rev : 16;
        UINT64 RX_4_Power : 11;
        UINT64 RX_3_Power : 11;
        UINT64 RX_2_Power : 11;
        UINT64 RX_1_Power : 11;
        UINT64 RX_Group : 4;
    } bit;
} TDUTRXPowereAnsMsg;

// EEPROM
typedef union tagDUTEEPROMReqMsg {
    UINT64 word;
    struct
    {
        UINT64 EEPROM_Req_Rev0 : 8;
        UINT64 EEPROM_Req_Rev1 : 8;
        UINT64 EEPROM_Req_Rev2 : 8;
        UINT64 EEPROM_Req_Rev3 : 8;
        UINT64 EEPROM_Req_Rev4 : 8;
        UINT64 EEPROM_Req_Rev5 : 8;
        UINT64 EEPROM_Req_Rev6 : 8;
        UINT64 EEPROM_Req_Rev7 : 8;
    } bit_big;
    struct
    {
        UINT64 EEPROM_Req_Rev7 : 8;
        UINT64 EEPROM_Req_Rev6 : 8;
        UINT64 EEPROM_Req_Rev5 : 8;
        UINT64 EEPROM_Req_Rev4 : 8;
        UINT64 EEPROM_Req_Rev3 : 8;
        UINT64 EEPROM_Req_Rev2 : 8;
        UINT64 EEPROM_Req_Rev1 : 8;
        UINT64 EEPROM_Req_Rev0 : 8;
    } bit;
} TDUTEEPROMReqMsg;

typedef union tagDUTEEPROMAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 EEPROM_Ans_Rev0 : 8;
        UINT64 EEPROM_Ans_Rev1 : 8;
        UINT64 EEPROM_Ans_Rev2 : 8;
        UINT64 EEPROM_Ans_Rev3 : 8;
        UINT64 EEPROM_Ans_Rev4 : 8;
        UINT64 EEPROM_Ans_Rev5 : 8;
        UINT64 EEPROM_Ans_Rev6 : 8;
        UINT64 EEPROM_Ans_Rev7 : 8;
    } bit_big;
    struct
    {
        UINT64 EEPROM_Ans_Rev7 : 8;
        UINT64 EEPROM_Ans_Rev6 : 8;
        UINT64 EEPROM_Ans_Rev5 : 8;
        UINT64 EEPROM_Ans_Rev4 : 8;
        UINT64 EEPROM_Ans_Rev3 : 8;
        UINT64 EEPROM_Ans_Rev2 : 8;
        UINT64 EEPROM_Ans_Rev1 : 8;
        UINT64 EEPROM_Ans_Rev0 : 8;
    } bit;
} TDUTEEPROMeAnsMsg;

typedef union tagDUTAngleMapperReqMsg {
    UINT64 word;
    struct
    {
        UINT64 TxCh            : 8;
        UINT64 RW              : 8;
        UINT64 measuredAngle   : 8;
        UINT64 realAngle       : 8;
        UINT64 res             : 8;
        UINT64 password        : 32;
    } bit_big;
    struct
    {
        UINT64 password        :24;
        UINT64 res             : 8;
        UINT64 realAngle       : 8;
        UINT64 measuredAngle   : 8;
        UINT64 RW              : 8;
        UINT64 TxCh            : 8;
    } bit;
} TDUAngleMapperReqMsg;

typedef union tagDUTAngleMapperAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 TxCh            : 8;
        UINT64 RW              : 8;
        UINT64 measuredAngle   : 8;
        UINT64 realAngle       : 8;
        UINT64 result          :32;
    } bit_big;
    struct
    {
        UINT64 result          :32;
        UINT64 realAngle       : 8;
        UINT64 measuredAngle   : 8;
        UINT64 RW              : 8;
        UINT64 TxCh            : 8;
    } bit;
} TDUTAngleMapperAnsMsg;

typedef union tagDUTAngleMapperEnReqMsg {
    UINT64 word;
    struct
    {
        UINT64 wr              : 8;
        UINT64 mode            : 8;
        UINT64 password        :24;
        UINT64 res             :24;
    } bit_big;
    struct
    {
    	UINT64 password        :24;
        UINT64 res             :24;
        UINT64 mode            : 8;
        UINT64 wr              : 8;
    } bit;
} TDUAngleMapperEnReqMsg;

typedef union tagDUTAngleMapperEnAnsMsg {
    UINT64 word;
    struct
    {
        UINT64 wr              : 8;
        UINT64 mode            : 8;
        UINT64 result          :24;
        UINT64 password        :24;
    } bit_big;
    struct
    {
        UINT64 result          :24;
        UINT64 password        :24;
        UINT64 mode            : 8;
        UINT64 wr              : 8;
    } bit;
} TDUTAngleMapperEnAnsMsg;

typedef union {
    UINT64 word;
    struct
    {
        UINT64 WR 		:8;	
        UINT64 CAL_MODE	:8;  //0: 一度一校准 1:曲线拟合
        UINT64 PROFILE 	:8;
    	UINT64 RESV		:40;
    } bit_big;	
    struct
    {
    	UINT64 RESV		:40;
        UINT64 PROFILE 	:8;
        UINT64 CAL_MODE	:8;  //0: 一度一校准 1:曲线拟合
        UINT64 WR 		:8;	
    } bit;
} TDUTAntCalModeReqAnsMsg;

/**
 * @brief
 * @param
 * @return
 * @warning
 */

typedef struct tagDUTCtrl
{
    INT8 isTestMode;
    INT8 isTestTrackObj;  //生产测试使用原始点还是跟踪后的点，测试原始点为0，测试跟踪后的点为1。
} TDUTCtrl;

typedef struct 
{
    uint8_t isTargetListMode;
    uint8_t is2DFFTMode;
    uint8_t targetListTrigger;//接收转台信号后改变的标志位，在发送一次波形以及目标列表信息后恢复invalid状态
    uint8_t fftTrigger;
    uint8_t targetListWaveTrigger;
    uint8_t fftWaveTrigger;
	uint8_t motorCaliDirect; //0：电机水平转动校准 [CAL_MOTOR_H_DIRECTION] 1：电机俯仰转动校准 [CAL_MOTOR_V_DIRECTION]
    int     angle;
    float   range;
    float   speed;
	uint8_t manualSwithProfileIdx;//手动模式下，由上位机切换的profile值
} FactoryFlag;

static TDUTCtrl gDUTCtrl = {DUT_MODE_CLOSE, DUT_OBJ_MODE_RAW};
static FactoryFlag gCheckFlag;
static UINT8 gDUTFrameCount = 0;
uint32_t cfg_fft_nve_rangeindex_start = 50;
uint32_t cfg_fft_nve_rangeindex_end = 70;

#define CTLRR_FACTORY_PASSWORD 0x3158AF


uint8_t get_dut_mode_next_profile_idx(void){
	static uint8_t nextProfileIdx = (DUT_CAL_MODE_PROFILE_NUM - 1);
	switch(gDUTCtrl.isTestMode){
		case DUT_MODE_CALI:
            nextProfileIdx = gCheckFlag.fftWaveTrigger 
                           ? (nextProfileIdx + 1) % DUT_CAL_MODE_PROFILE_NUM : DUT_CAL_MODE_PROFILE_NUM - 1;
		break;
		case DUT_MODE_NORMAL:
			nextProfileIdx = gCheckFlag.manualSwithProfileIdx;
		break;
		default:break;
	}
	return nextProfileIdx;
}

INT DUTGetTestState(void)
{
    return (gDUTCtrl.isTestMode);
}

INT8 DUTGetTestObjType(void)
{
	return (gDUTCtrl.isTestTrackObj);
}

INT DUTProcessCANRXFrame(uint8_t canIdx,TCANRxBuf *pBuf)
{
    char bbsn[7];
	int32_t result = 0; 
	uint64_t ack = 0;
	void *pAckMsg = NULL, *pRecvMsg = NULL;
    extern radar_config_t *getRadarCfg(void);
    radar_config_t *pRadarCfg = getRadarCfg();
	uint8_t wChannel = 0, wProfile = 0;
	
    //这里如果进行转换，只能进行bit的结构体操作，否则需要再将数据转换一次
    convert(pBuf->data);

    if (pBuf == 0){
        return FALSE;
    }

    if (gDUTCtrl.isTestMode == DUT_MODE_CLOSE){
        return ERROR;
    }

    TDUTSelfTestReqMsg *pDUTSelfTestReqMsg;
    TDUTSelfTestAnsMsg mDUTSelfTestAnsMsg;

    TDUAngleMapperReqMsg *AngleMapperMsg;
    TDUTAngleMapperAnsMsg AngleMapperAns;

    TDUAngleMapperEnReqMsg *AngleMapperEnRes;
    TDUTAngleMapperEnAnsMsg AngleMapperEnAns;
		
    switch (pBuf->id)
    {
        case FACTORY_CAN_SELFTEST_REQ:
            pDUTSelfTestReqMsg = (TDUTSelfTestReqMsg *)&pBuf->data[0];
            if (pDUTSelfTestReqMsg->bit.CMD_Mode == 1)
            {
    //            mDUTSelfTestAnsMsg.bit.ST_TEMP = DevMgrGetTemperature();
                mDUTSelfTestAnsMsg.bit.ST_MCU_ERR = 0;
                mDUTSelfTestAnsMsg.bit.ST_MMIC_ERR = 0;
                mDUTSelfTestAnsMsg.bit.ST_NVM_ERR = 0;
                mDUTSelfTestAnsMsg.bit.ST_PMIC_ERR = 0;
                mDUTSelfTestAnsMsg.bit.ST_Ans_Rev1 = 0x1f;
                mDUTSelfTestAnsMsg.bit.ST_Ans_Rev2 = 0xff;
                mDUTSelfTestAnsMsg.bit.ST_Ans_Rev3 = 0xff;
                mDUTSelfTestAnsMsg.bit.ST_Ans_Rev4 = 0xff;
                mDUTSelfTestAnsMsg.bit.ST_Ans_Rev5 = 0xff;
                mDUTSelfTestAnsMsg.bit.ST_Ans_Rev6 = 0xff;
                mDUTSelfTestAnsMsg.bit.ST_Ans_Rev7 = 0xff;
                gCAN[canIdx].sendFrame(&mDUTSelfTestAnsMsg.word, FACTORY_CAN_SELFTEST_ANS);
            }
            break;

        case FACTORY_CAN_OEM_RADAR_ID_REQ:
        {
            DUTRadarIdReqMsg *pRadarIdMsg = (DUTRadarIdReqMsg *)pBuf->data;
            DUTRadarIdAnsMsg msg;
            if (pRadarIdMsg->bit.RadarId_WR)
            {
                setCfgRadarId(pRadarIdMsg->bit.RadarId);
            }

            msg.bit.RadarId_Result = pRadarIdMsg->bit.RadarId_WR;
            msg.bit.RadarId = radar_config_using->compli_para.radarId;
            gCAN[canIdx].sendFrame(&msg.word, FACTORY_CAN_OEM_RADAR_ID_ANS);
            break;
        }
    		
        case FACTORY_CAN_WR_ANG_MAP_REQ:
            AngleMapperMsg = (TDUAngleMapperReqMsg *)&pBuf->data[0];
            int8_t realAngle_L,realAngle_H;
            int16_t Angle_u;
            uint8_t txCh = AngleMapperMsg->bit.TxCh;
            uint8_t wr = AngleMapperMsg->bit.RW;
            float measuredAngle = ((int8_t)AngleMapperMsg->bit.measuredAngle) / 1.0f;
            float realAngle = ((int8_t)AngleMapperMsg->bit.realAngle) / 1.0f;
            if(wr) {
                //write
                result = setAngleValue(txCh, measuredAngle, realAngle);
                EMBARC_PRINTF("\r\nwrite2 measuredAngle =%.2f , realAngle =%.2f  result =%d  txCh=%d\r\n",measuredAngle,realAngle,result,txCh);
            } else {
                //read
                AngleMapping(txCh, measuredAngle, &realAngle);
            }
            
            AngleMapperAns.bit.TxCh = txCh;
            AngleMapperAns.bit.RW = wr;
            AngleMapperAns.bit.measuredAngle = (int8_t)(measuredAngle);
            AngleMapperAns.bit.realAngle = (int8_t)(realAngle);
            if(result == RADAR_SUCCESS){
                AngleMapperAns.bit.result = 0x1; 
            }else{
                AngleMapperAns.bit.result = 0xFF; 
            }
    		gCAN[PCAN].sendFrame(&AngleMapperAns.word, FACTORY_CAN_WR_ANG_MAP_ANS);
            break;
        case FACTORY_CAN_EN_ANG_MAP_REQ:
            AngleMapperEnRes = (TDUAngleMapperEnReqMsg *)&pBuf->data[0];
            result = RADAR_FAILED;
            AngleMapperEnAns.bit.mode = 0xF;
            if(AngleMapperEnRes->bit.password == CTLRR_FACTORY_PASSWORD)
            {
            	EMBARC_PRINTF("== OK\r\n");
                if(AngleMapperEnRes->bit.wr)
                {
                    if(AngleMapperEnRes->bit.mode == 0 || AngleMapperEnRes->bit.mode == 1)
                    {
                        setCfgAngleMap(AngleMapperEnRes->bit.mode);
                        AngleMapperEnAns.bit.mode = AngleMapperEnRes->bit.mode;
                        result = RADAR_SUCCESS;
                    }
                    else
                    {
                        AngleMapperEnAns.bit.mode = 0xF;
                    }
                }
                else
                {
                    AngleMapperEnAns.bit.mode = radar_config_using->angleMapperEn;
                    result = RADAR_SUCCESS;
                }
            }
            AngleMapperEnAns.bit.password = 0;
            AngleMapperEnAns.bit.wr = AngleMapperEnRes->bit.wr;
            if(result == RADAR_SUCCESS)
            {
                AngleMapperEnAns.bit.result = 0x1; 
            }
            else
            {
                AngleMapperEnAns.bit.result = 0xFF; 
            }
            gCAN[canIdx].sendFrame(&AngleMapperEnAns.word, FACTORY_CAN_EN_ANG_MAP_ANS);
            break;

    	case FACTORY_CAN_SET_ANGLE_CAL_MOLE_REQ:
    		pRecvMsg = (TDUTAntCalModeReqAnsMsg *)&pBuf->data[0];
    		pAckMsg  = (TDUTAntCalModeReqAnsMsg *)&ack;	
    		((TDUTAntCalModeReqAnsMsg *)pAckMsg)->word = ((TDUTAntCalModeReqAnsMsg *)pRecvMsg)->word;
    		wProfile = ((TDUTAntCalModeReqAnsMsg *)pRecvMsg)->bit.PROFILE;
    		if(((TDUTAntCalModeReqAnsMsg *)pRecvMsg)->bit.WR == 0x01){ //写入
    			if(wProfile <  NUM_FRAME_TYPE){
    				pRadarCfg->antCaliMode[wProfile] = ((TDUTAntCalModeReqAnsMsg *)pRecvMsg)->bit.CAL_MODE;				
    				((TDUTAntCalModeReqAnsMsg *)pAckMsg)->bit.WR = 0x21;
    			}else{
    				((TDUTAntCalModeReqAnsMsg *)pAckMsg)->bit.WR = 0x00;	
    			}
    		}
    		gCAN[canIdx].sendFrame(pAckMsg, FACTORY_CAN_ANGLE_CAL_MOLE_ANS);
    	break;
    	
        default:
            return ERROR;
    }

    return SUCCESS;
}

