#ifndef TM01_UART_H
#define TM01_UART_H

#include <string.h>
#include "embARC_debug.h"
#include "embARC.h"



#define UART_CID_1			1
#define UART_CID_2			2
#define UART_CID_3			3
#define UART_CID_4			4
#define UART_CID_5			5
#define UART_CID_6			6
#define UART_CID_7			7




int32_t case_uart_1(uint32_t id, uint32_t len, uint32_t baud);
int32_t case_uart_2(uint32_t id, uint32_t len, uint32_t baud);
int32_t case_uart_34(uint32_t id, uint32_t len, uint32_t baud);
int32_t case_uart_5(uint32_t id, uint32_t len);
int32_t case_uart_6(uint32_t id, uint32_t len, uint32_t baud);
int32_t case_uart_7(uint32_t id, uint32_t len, uint32_t baud);
int32_t case_uart_8(uint32_t id, uint32_t len, uint32_t baud);
int32_t case_uart_10(uint32_t len);


#endif
