#ifndef CAN_SIGNAL_INTERFACE_H
#define CAN_SIGNAL_INTERFACE_H

#include "track.h"

#define SENSOR_ID                           0


/***************************CAN ID : RX************************* ****/
/* CAN OTA Message ID */
#define CAN_OTA_ID                          (0x111 + SENSOR_ID * 0x10)

/* CAN Dynamic Data Message ID */
#define SCAN_FRAME_ID                       (0x200 + SENSOR_ID * 0x10)

/* CAN HIL Message ID */
#define CAN_HIL_ID                          (0x203 + SENSOR_ID * 0x10)

/* CAN Sensor configuration Message ID */
#define CAN_SENSOR_CFG_ID                   (0x204 + SENSOR_ID * 0x10)

/* CAN LVDS Dump Message ID */
#define CAN_DUMP_ID                         (0x222 + SENSOR_ID * 0x10)


/***************************CAN ID : TX************************* ****/
/* CAN Tcp seg data send */
#define CAN_TCP_SEG_ID                      (0x205 + SENSOR_ID * 0x10)

#define HEAD_FRAME_ID0                      (0x300 + SENSOR_ID * 0x10)
#define HEAD_FRAME_ID1                      (0x301 + SENSOR_ID * 0x10)
#define BK_FRAME_ID0                        (0x400 + SENSOR_ID * 0x10)
#define BK_FRAME_ID1                        (0x401 + SENSOR_ID * 0x10)
#define BK_FRAME_ID2                        (0x402 + SENSOR_ID * 0x10)    /* Reserved */
#define AK_FRAME_ID0                        (0x500 + SENSOR_ID * 0x10)
#define AK_FRAME_ID1                        (0x501 + SENSOR_ID * 0x10)
#define AK_FRAME_ID2                        (0x502 + SENSOR_ID * 0x10)    /* Reserved */
#define FRAME_ID                            (0x600 + SENSOR_ID * 0x10)    /* can int */


/* CAN OTA Magic code and HS code */
#define CAN_OTA_COM_MAGIC_NUM               (0x341200ff)
#define CAN_OTA_COM_HS_CODE                 (0xff002143)

/* TCP MAGIC NUMBER */
#define TCP_CTRL_PACKAGE_MAGICNUMBER        (0xDEADCC)

/* when output data via CAN/CANFD, controlling the bk and ak frame ID 0/1/2 data output
* default is nothing output, need to choose output bk or ak data by uart command(track new_format)*/
extern bool output_bk_id0;
extern bool output_bk_id1;
extern bool output_bk_id2;    /* Reserved */

extern bool output_ak_id0;
extern bool output_ak_id1;
extern bool output_ak_id2;    /* Reserved */

void track_pkg_can_print(track_t *track);
void track_pkg_can_int_print(track_t *track);
int32_t can_scan_signal(uint8_t *data, uint32_t len);
int32_t can_dump_signal(uint8_t *data, uint32_t len);
int32_t can_cli_commands(void);

#endif /* CAN_SIGNAL_INTERFACE_H */

