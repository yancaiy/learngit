#include "tm01_uart.h"
#include "embARC_debug.h"
#include "embARC.h"
#include "validation.h"
#include "vmodule.h"

#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "mux.h"
#include "alps_dmu_reg.h"
#include "uart_hal.h"
#include "dma_hal.h"


#define UART_VALIDATION_CASE(cid, f_entry, f_exit) VALIDATION_CASE_DEFINE(uart, MODULE_UART_ID, cid, f_entry, f_exit)
#define UART_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_UART_ID, cid, f_init)

static void func_uart_dma_tx(void *params);
static void func_uart_dma_rx(void *params);


unsigned char case_uart_data[40] = {0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9};
static void func_uart_dma_tx(void *params)
{
	int32_t result = dma_release_channel((uint32_t)params);
	if (E_OK != result) {
			EMBARC_PRINTF("DMA channel release failed.\r\n");
	}
	//EMBARC_PRINTF("[%s] uart_dma_tx done \r\n", __func__);
}

static void func_uart_dma_rx(void *params)
{
	uart_dma_read(0, case_uart_data, 8, func_uart_dma_rx);
	uart_dma_write(0, case_uart_data, 8, func_uart_dma_tx);
	int32_t result = dma_release_channel((uint32_t)params);
	if (E_OK != result) {
			EMBARC_PRINTF("DMA channel release failed.\r\n");
	}
	//EMBARC_PRINTF("[%s] uart_dma_rx done \r\n", __func__);
}

int32_t uart_receive_test(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	validation_case_t *case_ptr = (validation_case_t *)self;

	EMBARC_PRINTF("Module id:%d, Case id:%d \r\n", case_ptr->mod_id, case_ptr->case_id);

	return result;
}


int32_t uart_send_test(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	validation_case_t *case_ptr = (validation_case_t *)self;

	EMBARC_PRINTF("Module id:%d, Case id:%d \r\n", case_ptr->mod_id, case_ptr->case_id);

	return result;
}

static void uart0_isr_callback(void *params)
{
	unsigned char data[512];
	uart_read(0, data, 16);//len =16
	uart_write(0, data, 16);	

}

static void uart1_isr_callback(void *params)
{
	unsigned char data[512];
	uart_read(1, data, 16);//len =16
	uart_write(1, data, 16);	

}

/*len equals sizeof(*data),Return data when len characters are received,*/
/*polling mode*/
uint8_t data_case1[4]={0,1,2,3};
int32_t case_uart_1(uint32_t id, uint32_t len, uint32_t baud)
{
	uart_init(id, baud);
	EMBARC_PRINTF("enter uart case1\r\n");
	uart_read(id, data_case1, len);
	uart_write(id, data_case1, len);
	return 0;
}


uart_xfer_t case_uart_xfer;
/*interrupt mode*/
int32_t case_uart_2(uint32_t id, uint32_t len, uint32_t baud)
{
	unsigned char data[512];
	uart_init(id, baud);
	EMBARC_PRINTF("enter uart case2\r\n");
	if(id == 0){
		uart_callback_register(id, uart0_isr_callback);
	}else{
		uart_callback_register(id, uart1_isr_callback);
	}
	DEV_BUFFER *case_rx_buf = case_uart_xfer.rx_buf;
	DEV_BUFFER_INIT(case_rx_buf, data, 512);
	uart_buffer_register(id, DEV_RX_BUFFER, case_rx_buf);

	DEV_BUFFER *case_tx_buf = case_uart_xfer.tx_buf;
	DEV_BUFFER_INIT(case_tx_buf, data, 512);
	uart_buffer_register(id, DEV_TX_BUFFER, case_tx_buf);
	
	uart_interrupt_enable(id, DEV_XFER, 1);//enable interrupt
	return 0;
}




/*dma mode,note:data[0] size is a byte,Python script shuould transfer a byte every time,Return data when len integer are received*/
int32_t case_uart_34(uint32_t id, uint32_t len, uint32_t baud)
{
	unsigned char data[4]= {1,2,3,4};

	dw_uart_t *dev_uart = NULL;
	dev_uart = (dw_uart_t *)uart_get_dev(id);

	uart_init(id, baud);
	int_disable(dev_uart->int_no);//disable interrupt
	chip_hw_mdelay(1000);
	while(1){
		//uart_dma_read(id, data, len, func_uart_dma_rx);
		uart_dma_write(id, data, len, func_uart_dma_tx);
		chip_hw_udelay(200000);
	}
	return 0;
}



/*intterrupt mode,same as case2*/
int32_t case_uart_5(uint32_t id, uint32_t len)
{
	unsigned char data[512];
	uint32_t i = 0;
	uint32_t baud_t[10]={9600,14400,19200,38400,56000,57600,115200,128000,230400,256000};
	
	while(i<10){
		uart_init(id, baud_t[i]);
		if(id == 0){
			uart_callback_register(id, uart0_isr_callback);
		}else{
			uart_callback_register(id, uart1_isr_callback);
		}
		DEV_BUFFER *case_rx_buf = case_uart_xfer.rx_buf;
		DEV_BUFFER_INIT(case_rx_buf, data, 512);
		uart_buffer_register(id, DEV_RX_BUFFER, case_rx_buf);

		DEV_BUFFER *case_tx_buf = case_uart_xfer.tx_buf;
		DEV_BUFFER_INIT(case_tx_buf, data, 512);
		uart_buffer_register(id, DEV_TX_BUFFER, case_tx_buf);

		uart_interrupt_enable(id, DEV_XFER, 1);//enable interrupt
		i++;
	}
	return 0;
 	
}



/*polling mode*/
int32_t case_uart_6(uint32_t id, uint32_t len, uint32_t baud)
{
	dw_uart_t *dev_uart = NULL;
	dev_uart = (dw_uart_t *)uart_get_dev(id);
	unsigned char data[512];
 	uart_init(id, baud);
	int_disable(dev_uart->int_no);//disable interrupt
	while(1){	
		uart_read(id, data, len);
		uart_write(id, data, len);
	}
	
	return 0;
}


/*interrupt mode*/
int32_t case_uart_7(uint32_t id, uint32_t len, uint32_t baud)
{
	unsigned char data[512];
 	uart_init(id, baud);
	if(id == 0){
		uart_callback_register(id, uart0_isr_callback);
	}else{
		uart_callback_register(id, uart1_isr_callback);
	}
	DEV_BUFFER *case_rx_buf = case_uart_xfer.rx_buf;
	DEV_BUFFER_INIT(case_rx_buf, data, 512);
	uart_buffer_register(id, DEV_RX_BUFFER, case_rx_buf);

	DEV_BUFFER *case_tx_buf = case_uart_xfer.tx_buf;
	DEV_BUFFER_INIT(case_tx_buf, data, 512);
	uart_buffer_register(id, DEV_TX_BUFFER, case_tx_buf);
	
	uart_interrupt_enable(id, DEV_XFER, 1);//enable interrupt
	return 0;
}




/*dma mode*/
int32_t case_uart_8(uint32_t id, uint32_t len, uint32_t baud)
{
	dw_uart_t *dev_uart = NULL;
	dev_uart = (dw_uart_t *)uart_get_dev(id);
	uart_init(id, baud);
	int_disable(dev_uart->int_no);//disable interrupt
	chip_hw_mdelay(1000);
	uart_dma_read(id, case_uart_data, 8, func_uart_dma_rx);
	return 0;
}



int32_t case_uart_10(uint32_t len)
{
	unsigned char data0[512] = {00,01,02,03,04,05,06,07};
	unsigned char data1[512] = {10,11,12,13,14,15,15,15};

	//io_mux_uart1_func_sel(IO_MUX_FUNC3);//TO DO: can1 pinmux to uart0
	uart_init(0, 115200);
	uart_write(0, data0, len);

	io_mux_uart0_func_sel(IO_MUX_FUNC1);
	uart_init(1, 3000000);
	uart_write(1, data1, len);	
	return 0;
}




/* register validation case table */
//UART_VALIDATION_CASE(UART_CID_1, uart_receive_test, NULL);

/* register validation system case table, it will be used to test under OS */

/*
UART_VALIDATION_SYS_CASE(UART_CID_1, uart_receive_test);
UART_VALIDATION_SYS_CASE(UART_CID_2, case_uart_1);
UART_VALIDATION_SYS_CASE(UART_CID_3, case_uart_2);
UART_VALIDATION_SYS_CASE(UART_CID_4, case_uart_5);
UART_VALIDATION_SYS_CASE(UART_CID_5, case_uart_6);
UART_VALIDATION_SYS_CASE(UART_CID_6, case_uart_7);
UART_VALIDATION_SYS_CASE(UART_CID_7, case_uart_10);
*/

//用注册机制，需在case中添加参数解析

