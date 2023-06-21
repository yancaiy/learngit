
#ifndef _SYSTEM_MISC_H_
#define _SYSTEM_MISC_H_

#include <string.h>


#define GPIO_ID0          25    //pin C1
#define GPIO_ID1          28    //pin D2
#define GPIO_ID2          11    //pin B4
#define GPIO_ID3          29    //pin C5
#define GPIO_TURNING_IN   19	//pin B8
#define GPIO_ERROR_IN	  24	//pin D1
#define GPIO_IGN_IN		  17	//pin D9
#define GPIO_POWER_ERR_IN 21	//pin F5
#define PGIO_POWER_ON_OUT 18	//pin C7
#define GPIO_PWM_0        2     //pin M4  PWM0
#define GPIO_SAFE_STATUS  1     //pin N1  PWM1

//SPI M1 相关引脚
#define SPI_M1_MISO       	13    	//pin J1
#define SPI_M1_MOSI       	14    	//pin J3
#define SPI_M1_SEL_0      	15    	//pin H5
#define SPI_M1_SEL_1        16    	//pin K2
//#define SPI_M1_SEL_2   	xx		//pin J5 --不可GPIO控制，没有提供gpio id
#define SPI_S0_CLK	  		20		//pin C9


void GPIO_init(void);
void system_reset(void);

#endif

