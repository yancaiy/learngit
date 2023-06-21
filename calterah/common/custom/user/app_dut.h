/*
 * @Author: your name
 * @Date: 2020-06-11 09:55:51
 * @LastEditTime: 2021-11-04 11:52:39
 * @LastEditors: fang yongjun
 * @Description: In User Settings Edit
 * @FilePath: app_dut.h
 */
/*
 * APP_DUT.h
 *
 *  Created on: 2018��5��7��
 *      Author: RonLiu
 */

#ifndef APP_APP_DUT_H_
#define APP_APP_DUT_H_

#include "can_trans.h"
//#include "app_uart_protoco.h"
#include "track.h"

#define DUT_MODE_CLOSE 0x0
#define DUT_MODE_NORMAL 0x1
#define DUT_MODE_CALI 0x2
#define DUT_MODE_DEBUG 0x3
#define DUT_ACCESS_REQ  	0x0E    //安全请求
#define DUT_ACCESS_CHECK 	0x0F   //安全确认
#define DUT_SOFTWARE_REBOOT 	0xFF   //生产模式软件重启

//生产测试目标类型
#define DUT_OBJ_MODE_INVALID 0x0
#define DUT_OBJ_MODE_NULL 0x1
#define DUT_OBJ_MODE_RAW 0x2
#define DUT_OBJ_MODE_TRACK 0x3

//生产测试校准类型
#define DUT_SELF_CALC_TYPE_INVALID 0x0 //无效
#define DUT_SELF_CALC_TYPE_ACC 0x1     //加速度计校准
//校准错误应答码
#define DUT_SELF_CALC_ERROR_TYPE_INVALID 0x0        //无效
#define DUT_SELF_CALC_ERROR_TYPE_OK 0x1             //测试OK
#define DUT_SELF_CALC_ERROR_TYPE_PASSWORD_ERROR 0x2 //密钥错误
#define DUT_SELF_CALC_ERROR_TYPE_CALC_ERROR 0x3     //测试错误
#define DUT_SELF_CALC_ERROR_TYPE_TYPE_ERROR 0x4     //测试类型错误

//老化测试配置
#define DUT_AGINT_TEST_INVALID 0x0
#define DUT_AGINT_TEST_EN 0x1
#define DUT_AGINT_TEST_CLOSE 0x2

//协议版本配置
#define CFG_PROTOCOL_VER_PASSWORD 0xA
#define CFG_PROTOCOL_VER_0 0x0
#define CFG_PROTOCOL_VER_1 0x1
#define CFG_PROTOCOL_VER_2 0x2

//速度解模糊回验功能
#define CFG_SPEED_VAGUE_CHECK_PASSWORD 0xE
#define CFG_SPEED_VAGUE_CHECK_EN 0x0
#define CFG_SPEED_VAGUE_CHECK_CLOSE 0x1

//测试命令密钥
#define DUT_TEST_CMD_PASSWORD 0x3158AF00
//测试类型
#define DUT_TEST_CMD_TYPE_WTD 0x1
#define DUT_TEST_CMD_TYPE_OTHER 0x2
//...
//测试应答码
#define DUT_TEST_CMD_ANS_NUMBER_OPEN 0x1           //已启动测试
#define DUT_TEST_CMD_ANS_NUMBER_ERROR 0x2          //测试错误
#define DUT_TEST_CMD_ANS_NUMBER_PASSWORD_ERROR 0x3 //密钥错误
#define DUT_TEST_CMD_ANS_NUMBER_TYPE_ERROR 0x4     //测试类型错误
#define DUT_TEST_CMD_ANS_NUMBER_OTHER 0x5          //其他

#define CAL_MODE_ONE_REG_OEN_CHECK 0   // 1度1校准，默认
#define CAL_MODE_POS_AND_COPS 1        // 拟合的方式

#define CAL_MOTOR_H_DIRECTION  0  //角度校准 水平转动
#define CAL_MOTOR_V_DIRECTION  1  //角度校准 俯仰转动

extern INT DUTGetTestState(void);
extern INT8 DUTGetTestObjType(void);
extern INT DUTProcessCANRXFrame(uint8_t canIdx,TCANRxBuf *pBuf);
extern INT DUTUploadTargetCplx(INT ID, UINT64 dfft[], INT x, INT y);
extern INT DUTUploadTargetDfft(int id, uint32_t *dfft, int r, int v);
extern INT DUT_ant_calib_process(track_t *track);
extern INT DUT_ant_calib_process220Plus(track_t *track);
extern void sendAgingInfo(void);
extern uint8_t get_calib_list_mode();
extern uint8_t get_calib_fft_mode();
extern uint8_t get_calib_list_wave_trigger_flag();
extern uint8_t get_calib_fft_wave_trigger_flag();
extern uint8_t get_calib_list_trigger_flag();
extern uint8_t get_calib_fft_trigger_flag();
extern void set_calib_list_wave_trigger_flag(uint8_t flag);
extern void set_calib_list_trigger_flag(uint8_t flag);
extern void set_calib_fft_wave_trigger_flag(uint8_t flag);
extern void set_calib_fft_trigger_flag(uint8_t flag);
extern uint8_t get_calib_motor_direction(void);
extern uint8_t get_dut_mode_next_profile_idx(void);//for 220 plus
extern uint8_t get_dut_mode_manual_profile_idx(void);//for 220 plus

#endif /* APP_APP_DUT_H_ */
