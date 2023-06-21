#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "mux.h"
#include "alps_dmu_reg.h"
#include "gpio_hal.h"
#include "tm02_gpio.h"
#include "validation.h"
#include "vmodule.h"

#define GPIO_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_GPIO_ID, cid, f_init)

int32_t func_print_cnt = 0;
static void func_print()
{
	func_print_cnt++;
	EMBARC_PRINTF("[%s] enter isr \r\n", __func__);
}

static void pin_sel_gpio(uint32_t gpio_no)
{
	clock_enable(IO_MUX_CLOCK,1);
	if(((gpio_no >=0) && (gpio_no <=5)) || (gpio_no == 16) || (gpio_no == 17) || (gpio_no == 22) || (gpio_no == 23)){
		raw_writel(REL_REGBASE_IOMUX + gpio_no * 4,5);
	}
	if(((gpio_no >= 6) && (gpio_no <= 15)) || ((gpio_no >=18) && (gpio_no <= 21))){
		raw_writel(REL_REGBASE_IOMUX + gpio_no * 4,4);
	}
}
/*case 1,case2*/
void case_gpio_id12(uint32_t gpio_no, uint32_t direction, uint32_t val) 
{
	/*need enable iomux,and io mux first*/
	int32_t result = E_OK;
	gpio_init();
	pin_sel_gpio(gpio_no);
	/*0:input;1:output*/
	gpio_set_direct(gpio_no, direction);

	if (direction == 0) {
		result = gpio_read(gpio_no);
		EMBARC_PRINTF("[%s] result:%d\r\n", __func__, result);
	} else {
		result = gpio_write(gpio_no, val);
		if (E_OK != result) {
			EMBARC_PRINTF("[%s] failure, result:%d\r\n", __func__, result);
		}else{
			EMBARC_PRINTF("[%s] sucess, result:%d\r\n", __func__, result);
		}
	}
}

/*case 3,case4,case5,case6,case7*/

void case_gpio_id345(uint32_t gpio_no, gpio_int_active_t type)
{
	/*need enable iomux,and io mux first*/
	gpio_init();
	pin_sel_gpio(gpio_no);
	gpio_int_register(gpio_no, func_print, type);
	chip_hw_mdelay(1000);
}

/*case8,debounce*/
void case_gpio_debounce(void)
{
	raw_writel(0xa0100428, 0x1); // enable io mux clock.
	raw_writel(0xa0400048, 0x4); // io mux can1_tx-gpio18
	raw_writel(0xa0400038, 0x4); // SPI_S0_mosi-gpio14

	//close gpio debounce clock
	raw_writel(0xa0100460,0x0);

	//close debounce clock cnt
	raw_writel(0xa0100204,0x0);
	
	//set debounce cnt,100 times of reset value
	raw_writel(0xa0100138,0x2710);

	//load debounce
	raw_writel(0xa0100304,0x1);

	//enable debounce clock cnt
	raw_writel(0xa0100204,0x1);

	//enable gpio debounce clock
	raw_writel(0xa0100460,0x1);

	case_gpio_id345(14, GPIO_INT_RISING_EDGE_ACTIVE);//case 5

	//enable debounce
	raw_writel(0xa1800048, 0xffff);

	gpio_set_direct(23, 1);
	gpio_write(23, 0);
	int cnt = 0;
	while(1)
		{
		cnt++;
		EMBARC_PRINTF(" cnt = %d\r\n",cnt);
		gpio_write(23, 1);
		chip_hw_mdelay(1000);//1s
		gpio_write(23, 0);

		chip_hw_mdelay(2000);//2s
		EMBARC_PRINTF("rising edge comming\r\n");
		gpio_write(23, 1);
		chip_hw_mdelay(1000);//1s

		EMBARC_PRINTF("[main] interrupt status: %x\r\n",raw_readl(0xa1800044));
		EMBARC_PRINTF("falling edge comming\r\n");
		gpio_write(23, 0);
		chip_hw_mdelay(2000);//2s
		EMBARC_PRINTF("[main] interrupt status: %x\r\n",raw_readl(0xa1800044));
		chip_hw_mdelay(1000);

		}

}
int32_t gpio_get_value(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	validation_case_t *case_ptr = (validation_case_t *)self;

	EMBARC_PRINTF("Module id:%d, Case id:%d \r\n", case_ptr->mod_id, case_ptr->case_id);

	return result;
}


GPIO_VALIDATION_SYS_CASE(GPIO_CID_1, gpio_get_value);
