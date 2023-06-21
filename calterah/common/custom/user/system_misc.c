
#include "embARC.h"
#include "embARC_debug.h"
#include "FreeRTOS.h"
#include "gpio_hal.h"
#include "dw_gpio.h"
#include "radio_ctrl.h"
#include "system_misc.h"

void GPIO_init(void)
{
	gpio_set_direct(GPIO_ID0, DW_GPIO_DIR_INPUT);         
	gpio_set_direct(GPIO_ID1, DW_GPIO_DIR_INPUT);          
	gpio_set_direct(GPIO_ID2, DW_GPIO_DIR_INPUT);          
	gpio_set_direct(GPIO_ID3, DW_GPIO_DIR_INPUT);          
	//gpio_set_direct(GPIO_TURNING_IN, DW_GPIO_DIR_INPUT);   
	gpio_set_direct(GPIO_ERROR_IN, DW_GPIO_DIR_INPUT);
	gpio_set_direct(GPIO_PWM_0, DW_GPIO_DIR_INPUT); 
	gpio_set_direct(GPIO_SAFE_STATUS, DW_GPIO_DIR_INPUT);  	 
	//gpio_set_direct(GPIO_IGN_IN, DW_GPIO_DIR_INPUT); 		 
	gpio_set_direct(GPIO_POWER_ERR_IN, DW_GPIO_DIR_INPUT); 

	//SPI M1 相关改为输入-为降低功率
	gpio_set_direct(SPI_M1_MISO,DW_GPIO_DIR_INPUT);
	gpio_set_direct(SPI_M1_MOSI,DW_GPIO_DIR_INPUT);
	gpio_set_direct(SPI_M1_SEL_0,DW_GPIO_DIR_INPUT);
	gpio_set_direct(SPI_M1_SEL_1,DW_GPIO_DIR_INPUT);
	gpio_set_direct(SPI_S0_CLK,DW_GPIO_DIR_INPUT);
}

void system_reset(void)
{
	fmcw_radio_reset();

	while(1);
	return;
}

