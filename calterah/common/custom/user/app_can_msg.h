#ifndef __APP_CAN_MSG_H__
#define __APP_CAN_MSG_H__

#include "cfg.h"

#define PARAM_DATAMODE_RAW 0x0
#define PARAM_DATAMODE_TRACK 0x1
#define PARAM_DATAMODE_RAW_TRACK 0x2
#define PARAM_DATAMODE_NONE 0x3

#define  START_INFO_BASE (0x000)
#define QUERY_INFO_BASE (0x110)
#define SEND_INFO_BASE (0x210)
#define SET_TIMESTAMP_BASE (0x220)
#define SEND_RADAR_STATUE_INFO (0x230)

#define RADAR_VERSION_BASE (0x700)

#define SET_RADAR_CFG_BASE (0x200)
#define SET_RADAR_MODE_BASE (0x300)

#define SET_INSTALL_INFO_BASE (0x400) //配置安装信息

#define RADAR_SNY_INFO_BASE (0x500) //输出同步的信息-暂时调试用
#define FRAME_END_BASE (0x5F0)

#define TRK_OBJ_START_BASE (0x6A0)
#define TRK_OBJ_INFO_BASE (0x6B0)
#define TRK_OBJ_EXTEND_1_BASE (0x6C0)
#define TRK_OBJ_EXTEND_2_BASE (0x6D0)

#define RAW_OBJ_START_BASE (0x600)
#define RAW_OBJ_INFO_BASE (0x710)
#define RAW_OBJ_HEIGH_BASE (0x720)

#define FCW_TIME_INFO_1_BASE (0x6E0)
#define FCW_TIME_INFO_BASE (0x6F0)
#define FCW_WARNING_INFO_BASE (0x7F0)

#define RADAR_SPEED_INFO_BASE (0x7E0)

#define RAW_VEL_OBJ_START_BASE (0x680)
#define RAW_VEL_OBJ_INFO_BASE  (0x690)

//与雷达ID不挂钩
#define RADAR_STATUS        0x442       //雷达状态
#define ABS_ESP_1           0x2E9       //凯翼车速信息
#define YAS_1               0x127       //凯翼偏航率
#define ICM_DateTime        0x453       //凯翼时间
#define EPS_02              0x2A4       //凯翼方向盘转角

#define SET_CAR_INFO 		0x3F0       //输入车速信息  -- 大端模式
#define SET_CAR_INFO_2 		0x4F5       //输入车速信息  -- 大端模式 -- 》放空，不能使用
#define TCU_2_CVT_DCT       0x300       //凯翼挡位信息（可得车速的正负）
#define ABS_ESP_1           0x2E9       //凯翼车速信息
#define YAS_1               0x127       //凯翼偏航率
#define EMS_2               0x101       //凯翼引擎状态
#define ICM_DateTime        0x453       //凯翼时间
#define ICM_1               0x430       //凯翼ICM_1（包括：车辆里程、车速、电压值信息）
#define BCM_1_Door_Light    0x391       //凯翼钥匙信号

#define CMD_CAN_ANS_TEST 	0x00E       //CAN通信测试
#define CMD_CASCADE_ORDAER 	0x00F       //屏蔽\复位\解锁 雷达操作 -- 级联操作指令
//#define SET_res        		0x3F2 //传感器配置2
//#define SET_res        		0x3F3 //传感器配置3
//#define SET_res        		0x3F4 //传感器配置4
#define SET_CAR_VEL_INTEL 0x3F5     //车辆信息同步及输入
#define SET_CAR_YAWRATE_INTEL 0x3F6 //车辆偏航角信息输入
//#define SET_res        		0x3F7 //寄存器读命令
//#define SET_res        		0x3F8 //寄存器写命令

#if ADC_DUMP
/* - adc dump data cmd - set dump frame cnt */
#define SET_ADC_DUMP_FRAME_CNT  0x3F9
#endif /* ADC_DUMP */
#if HIL_FUNC
/* - hil func data cmd - set dump frame cnt */
#define SET_HIL_FUNC_FRAME_CNT  0x3FA
#define SET_HIL_FUNC_RADAR_REBOOT   0x3FB
#define SET_HIL_FUNC_FRAME_INTERVAL 0x3FC
#endif /* HIL_FUNC */

#define CAN_MSG_ID(id) (id + radar_config_using->compli_para.radarId)

#define START_INFO CAN_MSG_ID(START_INFO_BASE)
#define QUERY_INFO CAN_MSG_ID(QUERY_INFO_BASE)
#define SEND_INFO CAN_MSG_ID(SEND_INFO_BASE)
#define SEND_RADAR_STATUE CAN_MSG_ID(SEND_RADAR_STATUE_INFO)

#define RADAR_VERSION CAN_MSG_ID(RADAR_VERSION_BASE)

#define SET_TIMESTAMP CAN_MSG_ID(SET_TIMESTAMP_BASE)

#define SET_RADAR_CFG CAN_MSG_ID(SET_RADAR_CFG_BASE)
#define SET_RADAR_MODE CAN_MSG_ID(SET_RADAR_MODE_BASE)

#define SET_INSTALL_INFO CAN_MSG_ID(SET_INSTALL_INFO_BASE) //配置安装信息

#define RADAR_SNY_INFO CAN_MSG_ID(RADAR_SNY_INFO_BASE) //输出同步的信息-暂时调试用
#define FRAME_END CAN_MSG_ID(FRAME_END_BASE)

#define TRK_OBJ_START CAN_MSG_ID(TRK_OBJ_START_BASE)
#define TRK_OBJ_INFO CAN_MSG_ID(TRK_OBJ_INFO_BASE)
#define TRK_OBJ_EXTEND_1 CAN_MSG_ID(TRK_OBJ_EXTEND_1_BASE)
#define TRK_OBJ_EXTEND_2 CAN_MSG_ID(TRK_OBJ_EXTEND_2_BASE)

#define RAW_OBJ_START CAN_MSG_ID(RAW_OBJ_START_BASE)
#define RAW_OBJ_INFO CAN_MSG_ID(RAW_OBJ_INFO_BASE)
#define RAW_OBJ_HEIGH CAN_MSG_ID(RAW_OBJ_HEIGH_BASE)

#define FCW_TIME_INFO_1 CAN_MSG_ID(FCW_TIME_INFO_1_BASE)
#define FCW_TIME_INFO CAN_MSG_ID(FCW_TIME_INFO_BASE)
#define FCW_WARNING_INFO CAN_MSG_ID(FCW_WARNING_INFO_BASE)

#define RADAR_SPEED_INFO CAN_MSG_ID(RADAR_SPEED_INFO_BASE)

#define RAW_VEL_OBJ_START CAN_MSG_ID(RAW_VEL_OBJ_START_BASE)
#define RAW_VEL_OBJ_INFO CAN_MSG_ID(RAW_VEL_OBJ_INFO_BASE)



//获取Flash值命令
#define GET_FLASH_VALUE CAN_MSG_ID(0x310)
#define CFG_FLASH_CMD CAN_MSG_ID(0x320)          //写flash命令
#define SET_CALC_INFO CAN_MSG_ID(0x410)          //配置安装校准信息
#define SEND_CALC_INFO CAN_MSG_ID(0x420)         //回复校准信息
#define GET_CALC_INFO SEND_CALC_INFO             //获取校准结果
#define SET_SELF_CALC_INFO CAN_MSG_ID(0x430)     //配置自校准信息
#define SET_CALC_EXT_CFG   CAN_MSG_ID(0x440) //安装校准扩展配置接口
#define SET_ACC_SELF_CALC_INFO CAN_MSG_ID(0x450) //加速度计校准
#define ACC_GYRO_PUT_INFO CAN_MSG_ID(0x460)      //加速度计、陀螺仪输出信息
#define SET_SERVICE_CALI_INFO CAN_MSG_ID(0x470)      //服务校准启动命令
#define SERVICE_CALI_INFO_DEB CAN_MSG_ID(0x480)      //服务校准进度信息以及校准目标等调试信息

//与雷达ID不挂钩
#define CMD_CASCADE_ORDAER 0x00F //屏蔽\复位\解锁 雷达操作 -- 级联操作指令
//#define SET_res        		0x3F2 //传感器配置2
//#define SET_res        		0x3F3 //传感器配置3
//#define SET_res        		0x3F4 //传感器配置4
#define SET_CAR_VEL_INTEL 0x3F5     //车辆信息同步及输入
#define SET_CAR_YAWRATE_INTEL 0x3F6 //车辆偏航角信息输入
//#define SET_res        		0x3F7 //寄存器读命令
//#define SET_res        		0x3F8 //寄存器写命令

//error Code
//error Code
#define RADAR_INIT_FAILED                       100     //雷达初始化失败
#define RADAR_ERROR_CODE_YAWRATE	  			101		//车速信息错误  曲率输入错误
#define RADAR_ERROR_CODE_CARSPEED	  			102		//车速信息错误 OBD车速输入错误
#define RADAR_ERROR_CODE_MMIC_TEMPERATURE	  	110		//MMIC芯片温度异常
#define RADAR_ERROR_CODE_MCU_TEMPERATURE	  	111		//MCU 芯片温度异常
#define RADAR_ERROR_FLASH_HEAD_CHECK_ERROR	  	112		//Flash Head Error
#define RADAR_ERROR_FLASH_CRC_CHECK_ERROR	  	113		//Flash CRC ERROR
#define RADAR_SEL_CALI_FALIED                   114     //雷达自标定失败
#define RADAR_ERROR_CODE_SELF_CALC_ANGLEOFFSET	120		//自标定角度异常
#define RADAR_ERROR_CODE_INSTALL_CALC_ANGLE		130		//安装标定角度异常

#define CMD_BOOT_MAGIC 0x55504752414445FE
#define CMD_WRITE_FLASH_MAGIC 0x55504752414445FE
#define CMD_READ_CFG 0xFFFFFFFFFFFFFFFF
#define PASSWORD_RADAR_MODE_SW 0x3158AF
#define CMD_CASCADE_ORDAER_PASSWORD 0x3158AF003158

#define CMD_GET_INSTALL_CALC_INFO_PASSWORD 0x0FFFFFFFFFFFFFFF
#define CMD_GET_SELF_CALC_INFO_PASSWORD 0x1FFFFFFFFFFFFFFF

#define CMD_CAN_ANS_TEST_MAGIC 		0xA1B2C3D4A5B6C7D8
#define CMD_CAN_ANS_TEST_MAGIC_ANS 	0xA8B7C6D5A4B3C2D1

#endif
