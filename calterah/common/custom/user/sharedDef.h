/*
 * @Author: your name
 * @Date: 2020-05-16 14:22:22
 * @LastEditTime: 2021-11-03 15:58:10
 * @LastEditors: fang yongjun
 * @Description: In User Settings Edit
 * @FilePath: sharedDef.h
 */
#ifndef __SHARED_DEF_H__
#define __SHARED_DEF_H__

#include "radarCfg.h"

//====================================================================
#define MAX_NB_OF_TARGETS       100          // Maximum number of targets
#define SAMPLES_PER_CHIRP       SAMPLE_CNT  // number of samples per chirp
#define RANGE_BIN_NUM           (SAMPLES_PER_CHIRP / 2)

#define UPLOAD_RADAR_DATA_4CHADC            0x5
#define UPLOAD_RADAR_DATA_TARGETS           0x6
#define UPLOAD_RADAR_DATA_RFFT              0x7
#define UPLOAD_RADAR_DATA_TARGET_1CHMAG     0x8
#define UPLOAD_RADAR_DATA_DFFT              0x9
#define UPLOAD_RADAR_DATA_DBF               0xA
#define DEBUG_TRACK_MODE                    0xB
#define NORMAL_TRACK_MODE                   0xC
#define UPLOAD_RADAR_DATA_OBJS              0xD
#define UPLOAD_RADAR_RAW_OBJS               0xE

#define UPLOAD_RADAR_DEFAULT_MODE   UPLOAD_RADAR_DATA_TARGETS

#define UNVALID_VEL 10000 //无效速度

//提供给initial_flag变量的宏定义
#define RADAR_INIT_FLAG_NULL 0      //暂时没定义
#define RADAR_INIT_FLAG_START 1     //初始化状态
#define RADAR_INIT_FLAG_OK 2        //完成初始化
#define RADAR_INIT_FLAG_ONE_FRAME 3 //完成一帧处理，但是可能没有完成多帧的处理，没有可以进行跟踪
#define RADAR_INIT_FLAG_TRACK 4     //可以进行track
#define RADAR_INIT_FLAG_TRACK_END 5 //已经完成上一次数据的跟踪

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define RADAR_SUCCESS (0)
#define RADAR_FAILED (1)

#define RADAR_FALSE (0)
#define RADAR_TRUE (1)

#define TX_CH1 1
#define TX_CH2 2
//下线标定状态码
#define CALC_INIT_VALID			0    //表示校准初始化
#define CALC_ING_VALID			1    //表示校准中
#define CALC_OK_NOUSE			2	 //校准OK但不适用【校准出的角度超过范围】   ---> 标靶未对准【角度偏差超过±2度】
#define CALC_ERROR_RES			3	 //预留，不使用
#define CALC_USING				4	 //校准已使用(但还未保存)
#define CALC_SAVED				5	 //校准已保存
#define CALC_SAVED_ERROR		6	 //保存失败 --》实际暂时不存在失败
#define CALC_ERROR_VALID		7	 //校准错误/超时 				   ---> 标定场地有干扰【标定区域内目标太多，需要更换场地】
#define CALC_OK					8	 //标定数据采集结束
#define CALC_ERROR_NO_OBJ		9	 //校准错误	 				   ---> 指定位置未放标靶【未检测到3m附近的目标】
#define CALC_USER_QUIT_ERROR	10   //校准错误-用户退出
#define CALC_OTHER_ERROR		11   //校准错误-其他错误

//售后服务校准状态码
#define SERVICE_CALC_INIT				0    //表示校准初始化
#define SERVICE_CALC_ING				1    //表示校准中
#define SERVICE_CALC_CILLECT_OK			2	 //校准数据采集完成
#define SERVICE_CALC_USING				3	 //校准已使用
#define SERVICE_CALC_SAVED				4	 //校准已保存
#define SERVICE_CALC_TIMEOUT			5	 //校准错误-超时
#define SERVICE_CALC_OUT_RANGE_ERROR	6	 //校准错误-角度超过范围
#define SERVICE_CALC_SIGNAL_ERROR		7	 //校准错误-信号错误 - 预留
#define SERVICE_CALC_YAWRATE_ERROR		8	 //校准错误-横摆角错误
#define SERVICE_CALC_HOST_SPEED_ERROR	9	 //校准错误-车速错误
#define SERVICE_CALC_USER_QUIT_ERROR	10	 //校准错误-用户退出
#define SERVICE_CALC_OTHER_ERROR		11	 //校准错误-其他



#endif
