/*
 * @Author: your name
 * @Date: 2020-06-17 20:08:48
 * @LastEditTime: 2022-04-08 13:55:54
 * @LastEditors: fang yongjun
 * @Description: In User Settings Edit
 * @FilePath: can_dbg.h
 */
#ifndef CAN_DBG_H_
#define CAN_DBG_H_

#include "typedefs.h"

#define CAN_DBG_ID   0x145
#define CMD_FRAME_ID 0x114

typedef struct
{
    int cur_frame_type;
    int ch;
    int dat_type;
} CAN_DBG_T;

extern CAN_DBG_T can_dbg_mode;

int can_bb_datdump_serport_command_handler(int cur_frame_type, int ch, int dat_type);
int32_t can_cmd_headler_dbg_cmd(uint8_t canIdx, uint8_t *data, uint32_t len);
int32_t recv_can_cmd(uint8_t canIdx, uint8_t *data, uint32_t len);
void canPrintf(uint8_t canIdx, const char*fmt, ...);
uint8_t isCfgMode(void);

#endif
