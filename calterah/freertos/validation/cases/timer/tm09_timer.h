/*
 * @Descripttion:
 * @version:
 * @Author: calterah
 * @Date: 2021-08-30 10:12:52
 * @LastEditors: haozhonglei
 * @LastEditTime: 2021-08-31 13:54:59
 */
#ifndef TIMER_TEST_H
#define TIMER_TEST_H


typedef struct {

    uint32_t id; // 0~3;
    uint32_t mode;//
    uint32_t period_ms;
    uint32_t times;
    uint32_t reserve;

} timer_auto_test_config_t;

#define TIMER_CID_1     1
#define TIMER_CID_2     2
#define TIMER_CID_3     3
#define TIMER_CID_4     4

int32_t timer_test(uint32_t id,uint32_t mode,uint32_t period_ms,uint32_t times);

#endif