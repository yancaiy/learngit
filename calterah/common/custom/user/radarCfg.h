#ifndef __RADAR_CFG_H__
#define __RADAR_CFG_H__

#include "can_obj.h"

//--------------------------------------------------------
// 基带板配置
//--------------------------------------------------------
#define MCU_VB 1
#define MCU_VC 2
#define MCU_ALPS 3

#define BB_TYPE MCU_ALPS

//--------------------------------------------------------
// 天线配置
//--------------------------------------------------------
#define RX_ACTIVE 4           //接收天线数
#define ANT_INSTALL_REVERSE 0 //雷达倒装
#define ANT_DISTANCE_066 0    //0.66 lambda等间距天线
#define FURCAL_TX_ANT 0       //TX分叉天线
#define UNUNIFORM_COH_ANT 0   //非等间距相干天线
#define ANT_DISTANCE_5 0      //0.5 lambda等间距天线
#define RX_METER_HEIGH 0      //RX测高天线
#define TX_METER_HEIGH 0      //TX测高天线

#define TX_400_ANT_TYPE		0
#define TX_410_ANT_TYPE_B	1
#define TX_410_ANT_TYPE_A1	2
#define TX_410_ANT_TYPE_A2	3
#define TX_400C_ANT_TYPE    4
#define TX_410_1_ANT_TYPE	5
#define TX_430_ANT_TYPE     6
#define TX_420_1_ANT_TYPE	7  //实际与 TX_410_1_ANT_TYPE 一致 ,协议不一样
#define TX_440_ANT_TYPE		8
#define TX_220P_ANT_TYPE    9
#define TX_ANT_TYPE			TX_220P_ANT_TYPE

//--------------------------------------------------------
// 功能配置
//--------------------------------------------------------

#define RAW_TARGET_NUM          128     
#define RAW_TARGET_NUM_SHORT    64     //

//看门狗是否启用
#define USE_WATCHDOG 0

//跟踪插值功能是否启用
#define TRK_INTERPOLATION 1

//陀螺仪功能是否启用
#define GYRO_FUNCTION_EN 	0

//DV测试
#define DV_TEST_MODE_ENABLE 0 //DV测试模式协议

//服务校准使能开关
#define RADAR_SERVICE_ALIGNMENT_MODE_DBAA		1	//DBAA算法

//速度解模糊
#define FIX_VELOCITY_ENABLE 1

//接模糊回验功能
#define SPEED_VAGUE_CHECK_EN 1

//输出原始点相位信息使能
#define RADAR_RAWONJ_PHASE_OUTPUT_EN	0

//距离速度插值提升精度功能
//#define RANGE_VELOCITY_INTERPOLATION_FUNEN	1

//角度差值补偿
#if TX_ANT_TYPE == TX_400C_ANT_TYPE || TX_ANT_TYPE == TX_430_ANT_TYPE || TX_ANT_TYPE ==  TX_420_1_ANT_TYPE
#define ANGLE_MAP_EN 1
#else
#define ANGLE_MAP_EN 0
#endif

//自标定结果是否生效保存
#define AUTO_CALI_RESULT_ACTIVE  0

//是否支持busoff慢恢复
#define CAN_BUSOFF_SLOW_RECOVERY 0

//暗室测试解速度模糊功能  0：正常模式  1：关闭速度回验，关闭一些不关注点的过滤
#define FIX_VELOCITY_TEST 0

//协议类型
enum PROTOCOL_VER_E
{
    PROTOCOL_VER_0 = 0,  //距离精度 0.2   速度精度 0.2   角度精度 0.25
    PROTOCOL_VER_1 = 1,  //距离精度 0.1   速度精度 0.1   角度精度 0.25
    PROTOCOL_VER_2 = 2,  //距离精度 0.05  速度精度 0.1   角度精度 0.25
    PROTOCOL_VER_3 = 3,  //距离精度 0.2   速度精度 0.2   range存储的是y值，angle存储的是x值
    PROTOCOL_VER_4 = 4,  //距离精度 0.2   速度精度 0.2，  笛卡尔坐标
	PROTOCOL_VER_5 = 5,  //距离精度 0.05  速度精度 0.1    角度精度
    PROTOCOL_VER_MAX,
};

//Can总线协议类型
#define CANBUS_PROTOCOL_VER_DEFAULT        0
#define CANBUS_PROTOCOL_VER_Project_410_QZ 1 //清智LRR 410项目协议版本 //为升科协议
#define CANBUS_PROTOCOL_VER_HW_MDC         2 //华为MDC
#define CANBUS_PROTOCOL_VER_CONTI          3 //大陆协议
#define CANBUS_PROTOCOL_VER_CONTI410       4 //大陆410协议

#define KAIYI_VCAN_TYPE                    0 //凯翼VCAN协议类型，0为本身，1为玛奇朵

#define CAN_TX_EN_MASK 	  (1 << PCAN | 1 << VCAN)  //默认CAN使能发送配置

//CAN配置
enum CAN_CFG
{
    CAN_CFG_NULL = 0,
    CAN_CFG_500K = 1,
    CAN_CFG_1000K = 2,
    CAN_CFG_MAX,
};

#if DV_TEST_MODE_ENABLE

#define CANBUS_PROTOCOL_VER_USING CANBUS_PROTOCOL_VER_Project_410_QZ      //CAN协议接口
#define DBF_READ_FROM_FLASH	      1                                       //0：采用曲线拟合方式   1：采用从一度一校准方式
#define RADAR_USE_CAN0_FD         0                                       //0:CAN     1:CANFD
#define RADAR_USE_CAN1_FD         0                                       //0:CAN     1:CANFD
#define RADAR_CAN0_NOMI_BAND      CAN_BAUDRATE_500KBPS
#define RADAR_CAN0_DATA_BAND      0                                       //默认0
#define RADAR_CAN1_NOMI_BAND      CAN_BAUDRATE_500KBPS
#define RADAR_CAN1_DATA_BAND      0                                       //默认0

#else

#define CANBUS_PROTOCOL_VER_USING CANBUS_PROTOCOL_VER_CONTI410            //CAN协议接口
#define DBF_READ_FROM_FLASH	      1                                       //0：采用曲线拟合方式   1：采用从一度一校准方式

#define RADAR_USE_CAN0_FD         0                                       //0:CAN     1:CANFD
#define RADAR_USE_CAN1_FD         0                                       //0:CAN     1:CANFD

#if RADAR_USE_CAN0_FD
#define RADAR_CAN0_NOMI_BAND      CAN_BAUDRATE_500KBPS
#define RADAR_CAN0_DATA_BAND      CAN_BAUDRATE_2MBPS
#else
#define RADAR_CAN0_NOMI_BAND      CAN_BAUDRATE_1MBPS
#define RADAR_CAN0_DATA_BAND      0                                       //默认0
#endif

#if RADAR_USE_CAN1_FD
#define RADAR_CAN1_NOMI_BAND      CAN_BAUDRATE_500KBPS
#define RADAR_CAN1_DATA_BAND      CAN_BAUDRATE_2MBPS
#else
#define RADAR_CAN1_NOMI_BAND      CAN_BAUDRATE_1MBPS
#define RADAR_CAN1_DATA_BAND      0                                       //默认0
#endif

#endif

#if CANBUS_PROTOCOL_VER_USING == CANBUS_PROTOCOL_VER_HW_MDC
#define PROTOCOL_VER_USING    PROTOCOL_VER_4 //目前使用协议
#else
#define PROTOCOL_VER_USING    PROTOCOL_VER_0 //目前使用协议
#endif

//Tx通道，分远近
#define TX_CH_LONG_SHORT_RANGE 0x0 //远距离天线+近距离
#define TX_CH_LONG_RANGE       0x1 //远距离天线
#define TX_CH_SHORT_RANGE      0x2 //近距离天线
#define TX_CH_FILTER_NULL      0x3 //不过滤
#define IS_EXIST_HEIGHT_INFO(SrMode)	((SrMode == 0x3) || (SrMode == 0x4) || (SrMode == 0x5))
#define IS_EXIST_DML_FUN(SrMode)		(SrMode == 0x3)

//output模式
#define RAW_OUTPUT_MODE_NORMAL			0x0   //正常输出，会速度解模糊
#define RAW_OUTPUT_MODE_1				0x1	  //分profile输出，每次固定29个
#define RAW_OUTPUT_MODE_2				0x2   //分profile输出，每个profile混在一次输出
#define RAW_OUTPUT_MODE_3				0x3   //分profile输出，每个profile中加分割线，数目不定

//目标数量限制
#if CANBUS_PROTOCOL_VER_USING ==  CANBUS_PROTOCOL_VER_Project_410_QZ
#define MAX_TARGET_NUM_SEND 32
#elif CANBUS_PROTOCOL_VER_USING ==  CANBUS_PROTOCOL_VER_CONTI
#define MAX_TARGET_NUM_SEND 128
#elif CANBUS_PROTOCOL_VER_USING ==  CANBUS_PROTOCOL_VER_CONTI410
#define MAX_TARGET_NUM_SEND 40
#else
#define MAX_TARGET_NUM_SEND 40
#endif

//--------------------------------------------------------------------
// 采样参数配置
//--------------------------------------------------------------------
#define SAMPLE_CNT         1024      //采样点数
#define CHIRPS_PER_FRAME   128       // number of chirps per frame

#define SAMPLE_CNT_2       1024      //采样点数
#define CHIRPS_PER_FRAME_2 128       // number of chirps per frame

//--------------------------------------------------------------------
// 信号处理参数配置
//--------------------------------------------------------------------
#define DBF_BEAM_NUM 64  //96    //DBF beam数
#define RX_DBF_MASK 0xF  //DBF所使用的天线掩码
#define DBF_OUT_ANGLES 2 //每个2DFFT目标最多输出的角度数

//--------------------------------------------------------------------
// 应用场景配置
//--------------------------------------------------------------------
#define NORMAL_MODE_APP 0
#define FCW_MODE        1
#define ACC_MODE        2
#define BSD_MODE        3
#define RCW_MODE        4
#define ETC_MODE        5
#define LDTR_MODE       6 //路达交通雷达
#define FAR_HUMAN_MODE  7 //远距离行人模式
#define AIWAYS_MODE     8 //爱驰
#define TRAFFIC_MODE    9 //交通模式

#define RADAR_MODE NORMAL_MODE_APP

//TX切换定义，位域的方式
enum TX_SWITCH_E
{
    NO_TX_SW,      //不切换
    TX1_C0 = 0x1,
    TX2_C0 = 0x2,
    TX1_C1 = 0x4,
    TX2_C1 = 0x8,
    TX1_C0_C1 = 0x5,
    TX2_C0_C1 = 0xA,
    TX_ALL_PROFILE = 0xF, //所有profile切换
};

#if RADAR_MODE == TRAFFIC_MODE
#define TX_4CH_SW  1
#define TX_SW_NUM  4
#define TX_PEOFILE_SW_TYPE 1 //0, 切chirp， 1 切天线
#else
#define TX_4CH_SW  0
#define TX_SW_NUM  2
#endif

//--------------------------------------------------------------------
// 各应用场景对应的参数配置
//--------------------------------------------------------------------


//--------------------------------------------------------
// ADC采集数据使能 -- default: disable
//--------------------------------------------------------
#define ADC_DUMP  1
#define HIL_FUNC  1
//#define SWITCH_CT_TO_VENUS 	1
#define ADC_DUMP_MAGIC_NUM  (0x43544354)    /* - CTCT */
//#define ECU_REBOOT_NORMAL   (0x0)

//--------------------------------------------------------
// 默认基本参数配置
//--------------------------------------------------------
#define MAX_RANGE 210 //最大有效距离

//默认FMCW参数配置
#if defined(FMCW_START_FREQ)
//使用自定义的
#elif SAMPLE_CNT == 512
#define FMCW_START_FREQ 76.2
#define FMCW_BANDWIDTH 0.300
#define FMCW_TRAMPUP 42
#elif SAMPLE_CNT == 1024
#define FMCW_START_FREQ 76.2
#define FMCW_BANDWIDTH 0.400
#define FMCW_TRAMPUP 45
#endif

#define FMCW_TDOWN 3
#define FMCW_TIDLE 4

#define ANT_DISTANCE_CAL                                                  	\
    {                                                                     	\
        {0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148, 0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148,},\
        {0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148, 0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148,},\
		{0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148, 0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148,},\
		{0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148, 0, 3.1415, 6.2832, 9.4248, 12.56637, 15.70796, 18.8495, 21.991148,},\
    } // 这个值表示方法为---：2*pi*d/波长,需要配置到alps的值是   (d/波长)
#define ANT_DISTANCE_CAL_Y     \
    {                          \
        0, 0, 0, 0, 0, 0, 0, 0 \
    } // 这个值表示方法为---：2*pi*d/波长,需要配置到alps的值是   (d/波长)

#define ANT_PHASE_CAL          \
	{\
		{ \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
		},\
		{ \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
		},\
		{ \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
		},\
		{ \
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
		},\
	}

//默认校准参数配置
#if defined(ANT_DISTANCE_CAL)

#else  //使用自定义的

#endif

//采样模式
#ifndef TX_SW_MODE
#define TX_SW_MODE TX_ALL_PROFILE
#endif

#ifndef TX_SEL
#define TX_SEL 2
#endif

//硬件版本号
#undef HW_VERSION
#if defined(HW_VERSION)
//使用自定义的
#elif ANT_DISTANCE_5
#define HW_VERSION \
    {              \
        1, 2, 2, 0 \
    }
#elif FURCAL_TX_ANT
#define HW_VERSION \
    {              \
        1, 2, 3, 0 \
    }
#elif UNUNIFORM_COH_ANT
#define HW_VERSION \
    {              \
        1, 2, 4, 0 \
    }
#elif RX_METER_HEIGH
#define HW_VERSION \
    {              \
        1, 2, 5, 0 \
    }
#elif (TX_ANT_TYPE	== TX_410_1_ANT_TYPE) || (TX_ANT_TYPE == TX_420_1_ANT_TYPE)
#define HW_VERSION \
    {              \
        3, 2, 1, 0 \
    }
#elif (TX_ANT_TYPE == TX_220P_ANT_TYPE)
#define HW_VERSION \
    {              \
        3, 0, 5, 0 \
    }
#elif (TX_ANT_TYPE == TX_440_ANT_TYPE)
#define HW_VERSION \
    {              \
        3, 2, 4, 0 \
    }
#else
#define HW_VERSION \
    {              \
        1, 2, 1, 0 \
    }
#endif

//版本号规则
#define SW_VER_P1 2

#if BB_TYPE == MCU_VB
#define SW_VER_P2 0
#elif BB_TYPE == MCU_VC
#define SW_VER_P2 5
#elif BB_TYPE == MCU_ALPS
    #if TX_ANT_TYPE == TX_410_1_ANT_TYPE
        #define SW_VER_P2 8

        #define SW_VERSION              \
        {                               \
            SW_VER_P1, SW_VER_P2, 0, 14 \
        }
    #elif TX_ANT_TYPE == TX_220P_ANT_TYPE
        #define SW_VER_P2 12

        #define SW_VERSION              \
        {                               \
            SW_VER_P1, SW_VER_P2, 1, 13 \
        }
	#elif TX_ANT_TYPE == TX_440_ANT_TYPE
        #define SW_VER_P2 11

        #define SW_VERSION              \
        {                               \
            SW_VER_P1, SW_VER_P2, 1, 8 \
        }
    #elif TX_ANT_TYPE == TX_430_ANT_TYPE
        #define SW_VER_P2 9

        #define SW_VERSION              \
        {                               \
            SW_VER_P1, SW_VER_P2, 0, 1 \
        }
	#elif TX_ANT_TYPE == TX_420_1_ANT_TYPE
		#define SW_VER_P2 10

		#define SW_VERSION              \
		{                               \
			SW_VER_P1, SW_VER_P2, 0, 11 \
		}
    #else
        #define SW_VER_P2 7

        #define SW_VERSION              \
        {                               \
            SW_VER_P1, SW_VER_P2, 10, 2 \
        }
    #endif
#endif

//软件版本号
#if !defined(SW_VERSION)
//使用自定义的
#define SW_VERSION                 \
    {                              \
        SW_VER_P1, SW_VER_P2, 0, 1 \
    }

#endif


#endif //__RADAR_CFG_H__
