#ifndef TM03_XIP_H
#define TM03_XIP_H

#include <string.h>
#include "embARC_debug.h"
#include "embARC.h"

/**
 * \name XIP validation test ID names
 * @{
 */
/* NORMAL_READ_FW_VER */
#define XIP_CID_1			1
/* NORMAL_ERASE */
#define XIP_CID_2			2
/* NORMAL_WRITE_WORD */
#define XIP_CID_3			3
/* NORMAL_WRITE_BYTE */
#define XIP_CID_4			4
/* AES_READ_FW_VER */
#define XIP_CID_5			5
/* AES_ERASE */
#define XIP_CID_6			6
/* AES_WRITE_WORD */
#define XIP_CID_7			7
/* AES_WRITE_BYT */
#define XIP_CID_8E			8


/* The switch ID of test API */
#define ID_1_READ		0 	/*!< macro name for XIP read */
#define ID_2_ERASE		1 	/*!< macro name for XIP erase */
#define ID_3_WRITE		2 	/*!< macro name for XIP write */

typedef enum {
	eBYTE = 0,
	eWORD
} eDATA_TYPE;


typedef struct xip_session {
	uint32_t type;
	uint32_t addr;
	uint32_t rx_buf[64];
	uint32_t tx_buf[64];
	uint32_t len;
} xip_session_t;

int32_t xip_test(uint32_t id, xip_session_t *package);

#endif
