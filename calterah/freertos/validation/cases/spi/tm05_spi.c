#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "arc.h"
#include "arc_builtin.h"
#include "arc_cache.h"
#include "alps_hardware.h"
#include "alps_clock.h"
#include "alps_timer.h"
#include "dev_common.h"
#include "dw_ssi_reg.h"
#include "dw_ssi.h"
#include "spi_hal.h"
#include "dma_hal.h"
#include "tm05_spi.h"
#include "device.h"
#include "instruction.h"
#include "flash.h"
#include "config.h"
#include "validation.h"
#include "vmodule.h"

//#define SPI_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_SPI_ID, cid, f_init, NULL)

static spi_xfer_desc_t spim_xfer_config =
{
	.clock_mode = SPI_CLK_MODE_0,
	.dfs = 32,
	.cfs = 0,
	.spi_frf = SPI_FRF_STANDARD,
	.rx_thres = 0,
	.tx_thres = 0,
};


DEV_BUFFER tferdata;
DEV_BUFFER rferdata;

DEV_BUFFER q_txferdata;
DEV_BUFFER q_rxferdata;

uint32_t write_buffer_w[512] = { 0 };
uint32_t read_buffer_w[512] = { 0 };

uint8_t write_buffer_b[512] = { 0 };
uint8_t read_buffer_b[512] = { 0 };

void cache_flush(void){
	_arc_aux_write(AUX_DC_FLSH, 1);
	while (_arc_aux_read(AUX_DC_CTRL) & (1 << 8));
}
void cache_invalidate(void) {
	_arc_aux_write(AUX_DC_IVDC, 1);
	while (_arc_aux_read(AUX_DC_CTRL) & DC_CTRL_FLUSH_STATUS);
}

/* device : 0 --> QSPI ; 1 --> SPIM */
void dma_handshake_select(uint8_t device) {

	raw_writel(0xa0000508, device);
	raw_writel(0xa000050c, device);

}

static void spim_int_tx_callback(void *params)
{
	spi_interrupt_enable(0, 1, 0);
	spi_interrupt_uninstall(0, 1);
	spi_databuffer_uninstall(0, 1);
}
static void spim_int_rx_callback(void *params)
{
	spi_interrupt_enable(0, 0, 0);
	spi_interrupt_uninstall(0, 0);
	spi_databuffer_uninstall(0, 0);
}

void spim_DMA_write_callback(void *params)
{}
void spim_DMA_read_callback(void *params)
{}

void spis_int_tx_callback(void *params)
{}

void spiq_int_tx_callback(void *params)
{}
void spiq_int_rx_callback(void *params)
{}
void spiq_DMA_write_callback(void *params)
{
	//raw_writel(REG_DW_SSI_SER_OFFSET + 0xa1000000, 0);
}
void spiq_DMA_read_callback(void *params)
{
	//raw_writel(REG_DW_SSI_SER_OFFSET + 0xa1000000, 0);
}

static void data_buffer_install_word(uint32_t *buffer, uint32_t len)
{
	uint32_t i = 0;
	uint32_t n = 1;

	if (len > 1024)
	{
		len = 1024;
	}

	for (i = 0;i < len;i++)
	{
		buffer[i] = n;
		n += 1;
	}

}

static void data_buffer_install_byte(uint8_t *buffer, uint32_t len)
{
	uint32_t i = 0;
	uint32_t n = 1;

	if (len > 1024)
	{
		len = 1024;
	}

	for (i = 0; i < len; i++)
	{
		buffer[i] = n;
		n += 1;
	}

}

int32_t spim_test_polling_case1(uint32_t id, uint32_t baud_rate, uint32_t *data, uint32_t len)
{
	int32_t result = E_OK;

	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}

		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		result = spi_read(id, data, len);
		if (E_OK != result) {
			break;
		}
		chip_hw_mdelay(5000);
		result = spi_write(id, data, len);
		if (E_OK != result) {
			break;
		}

	} while (0);

	return result;
}
int32_t spim_test_int_transfer_case2(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}

		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		DEV_BUFFER_INIT(&tferdata, writedata, len);
		result = spi_databuffer_install(id, 1, &tferdata);//1 ->tx , 0 ->rx
		if (E_OK != result) {
			break;
		}
		result = spi_interrupt_install(id, 1, spim_int_tx_callback);
		if (E_OK != result) {
			break;
		}

		result = spi_interrupt_enable(id, 1, 1);
		if (E_OK != result) {
			break;
		}
	} while (0);

	return result;
}
int32_t spim_test_int_receive_case2(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}

		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		DEV_BUFFER_INIT(&rferdata, readdata, len);
		result = spi_databuffer_install(id, 0, &rferdata);//1 ->tx , 0 ->rx
		if (E_OK != result) {
			break;
		}
		result = spi_interrupt_install(id, 0, spim_int_rx_callback);
		if (E_OK != result) {
			break;
		}
		raw_writel(0xa1300000 + REG_DW_SSI_DR_OFFSET(0), 0x55);//transfer 1 byte data to TX FIFO first
		raw_writel(0x0010 + 0xa1300000, 1);
		chip_hw_mdelay(1);
		raw_writel(0x0010 + 0xa1300000, 0);
		result = spi_interrupt_enable(id, 0, 1);
		if (E_OK != result) {
			break;
		}
	} while (0);

	return result;
}
#if 0
int32_t spim_test_polling_case5(uint32_t id, uint32_t baud_rate, uint32_t *writedata, \
	uint32_t *readdata, uint32_t len)
{
	int32_t result = E_OK;
	uint32_t truedata = 0;
	uint32_t falsedata = 0;
	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}

		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		result = spi_xfer(id, writedata, readdata, len);
		if (E_OK != result) {
			break;
		}

		for (uint32_t i = 0; i < len; i++) {
			if (writedata[i] == readdata[i]) {
				truedata++;
			}
			else {
				falsedata++;
			}
		}
		EMBARC_PRINTF("truedata = %d\n", truedata);
		EMBARC_PRINTF("falsedata = %d\n", falsedata);
		if (truedata == len)
		{
			result = E_OK;
		} else {
			result = E_SYS;
		}
	} while (0);

	return result;
}
#endif
int32_t spim_test_polling_flashwrite_case6(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len)
{
	int32_t result = E_OK;
	uint32_t cmd = 0x06;

	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}

		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		spi_write(id, &cmd, 1);
		spi_write(id, writedata, len);

	} while (0);

	return result;
}
int32_t spim_test_polling_flashread_case6(uint32_t id, uint32_t baud_rate, uint32_t *writedata, \
	uint32_t *readdata, uint32_t len)
{
	int32_t result = E_OK;


	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}
		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		result = spi_xfer(id, writedata, readdata, len);
		if (E_OK != result) {
			break;
		}


	} while (0);

	return result;
}
int32_t spim_test_flashchiperase(uint32_t id, uint32_t baud_rate)
{
	int32_t result = E_OK;
	uint32_t cmd = 0x06;
	uint32_t erasecmd[4] = { 0xD8,0x00,0x00,0x00 };
	do {
		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}
		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		spi_write(id, &cmd, 1);
		spi_write(id, erasecmd, 4);

	} while (0);
	return result;
}

int32_t spiq_test_polling_flash_case7(uint32_t addr, uint8_t *wdata, uint8_t *rdata, uint32_t len)
{
	int32_t result = E_OK;

	result = flash_memory_erase(addr, len);

	result = flash_memory_writeb(addr, wdata, len);

	result = flash_memory_readb(addr, rdata, len);

	return result;
}

int32_t spiq_test_polling_flasherase_case7(uint32_t addr, uint32_t len)
{

	int32_t result = E_OK;

	result = flash_memory_erase(addr, len);

	return result;
}
int32_t spiq_test_int_transfer_case10(uint8_t *writedata, uint32_t len, uint32_t addr) {

	int32_t result = E_OK;

	do {

		DEV_BUFFER_INIT(&q_txferdata, writedata, len);
		qspi_databuffer_install(1, &q_txferdata, addr);//1 ->tx , 0 ->rx
		if (E_OK != result) {
			break;
		}
		qspi_interrupt_install(1, spiq_int_tx_callback);
		if (E_OK != result) {
			break;
		}

		qspi_interrupt_enable(1, 1);
		if (E_OK != result) {
			break;
		}
	} while (0);

	return result;
}
int32_t spiq_test_int_receive_case10(uint8_t *readdata, uint32_t len, uint32_t addr) {

	int32_t result = E_OK;

	do {
		DEV_BUFFER_INIT(&q_rxferdata, readdata, len);
		qspi_databuffer_install(0, &q_rxferdata, addr);//1 ->tx , 0 ->rx
		if (E_OK != result) {
			break;
		}
		qspi_interrupt_install(0, spiq_int_rx_callback);
		if (E_OK != result) {
			break;
		}
		qspi_interrupt_enable(0, 1);
		if (E_OK != result) {
			break;
		}
		/* make RX FIFO has 1 data ,then trigger RX ISR */
		raw_writel(8 + 0xa1000000, 0);
		raw_writel(4 + 0xa1000000, 1);
		raw_writel(8 + 0xa1000000, 1);
		raw_writel(0xa1000000 + 0x60, 0x03);
		raw_writel(0xa1000000 + 0x60, 0x0);
		raw_writel(0x10 + 0xa1000000, 1);
		chip_hw_udelay(10);
		raw_writel(0x10 + 0xa1000000, 0);
	} while (0);

	return result;
}

int32_t spiq_test_polling_flasherasewrite_loop_case15(uint32_t addr, uint8_t *wdata, uint32_t len)
{

	int32_t result = E_OK;

	for (uint32_t time = 0;time < 100;time++)
	{
		result = flash_memory_erase(addr, len);
		if (E_OK != result) {
			break;
		}

		chip_hw_mdelay(1);

		result = flash_memory_writeb(addr, wdata, len);
		if (E_OK != result) {
			break;
		}
	}

	return result;
}


int32_t spis_test_polling_read_case11(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}
		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		result = spi_read(id, readdata, len);

	} while (0);

	return result;
}
int32_t spis_test_polling_write_case11(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}
		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		spi_write(id, writedata, len);

	} while (0);

	return result;
}

int32_t spis_test_int_transfer_case2(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}

		spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		DEV_BUFFER_INIT(&tferdata, writedata, len);
		spi_databuffer_install(id, 1, &tferdata);//1 ->tx , 0 ->rx
		if (E_OK != result) {
			break;
		}
		spi_interrupt_install(id, 1, spis_int_tx_callback);
		if (E_OK != result) {
			break;
		}

		spi_interrupt_enable(id, 1, 1);
		if (E_OK != result) {
			break;
		}
	} while (0);

	return result;
}
int32_t spis_test_int_receive_case2(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}

		spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		DEV_BUFFER_INIT(&rferdata, readdata, len);
		spi_databuffer_install(id, 0, &rferdata);//1 ->tx , 0 ->rx
		if (E_OK != result) {
			break;
		}
		spi_interrupt_install(id, 0, spis_int_tx_callback);
		if (E_OK != result) {
			break;
		}
		spi_interrupt_enable(id, 0, 1);
		if (E_OK != result) {
			break;
		}
	} while (0);

	return result;
}

//SPI_DMAtest_API
int32_t spim_test_DMA_write_case4(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		dma_handshake_select(1);

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}
		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}
		result = dma_init();
		if (E_OK != result) {
			EMBARC_PRINTF("dma init fail\r\n");
			break;
		}
		cache_flush();

		result = spi_dma_write(id, writedata, len, spim_DMA_write_callback);
		if (E_OK != result) {
			break;
		}
	} while (0);

	return result;
}
int32_t spim_test_DMA_read_case4(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len)
{
	int32_t result = E_OK;

	do {

		dma_handshake_select(1);

		result = spi_open(id, baud_rate);
		if (E_OK != result) {
			break;
		}
		result = spi_transfer_config(id, &spim_xfer_config);
		if (E_OK != result) {
			break;
		}

		result = dma_init();
		if (E_OK != result) {
			EMBARC_PRINTF("dma init fail\r\n");
			break;
		}
		result = spi_dma_read(id, readdata, len, spim_DMA_read_callback);
		if (E_OK != result) {
			break;
		}
		cache_flush();
		cache_invalidate();
		chip_hw_mdelay(10);

	} while (0);

	return result;
}

/* QSPI DMA test API  : write/read LEN MAX = 1 Page SIZE = 256 */
int32_t spiq_test_dma_write_case11(uint32_t addr, uint32_t *wdata, uint32_t len)
{
	int32_t result = E_OK;

	dma_handshake_select(0);

	cache_flush();

	if (len > 256) { len = 256; }

	dma_init();

	result = qspi_dma_write(addr, wdata, len, NULL);


	return result;
}

int32_t spiq_test_dma_read_case11(uint32_t addr, uint32_t *rdata, uint32_t len)
{
	int32_t result = E_OK;

	dma_handshake_select(0);

	if (len > 256) { len = 256; }

	dma_init();

	result = qspi_dma_read(addr, rdata, len, NULL);

	dcache_invalidate();
	chip_hw_mdelay(10);

	return result;
}

/* SPI/QSPI auto test */
auto_test_param_t spi_test_config;

void IO_MUX_SPIM(void)
{
	iomux_cfg_12(1);
	iomux_cfg_13(1);
	iomux_cfg_14(1);
	iomux_cfg_15(1);
}
void IO_MUX_SPIS(void)
{
	iomux_cfg_12(0);
	iomux_cfg_13(0);
	iomux_cfg_14(0);
	iomux_cfg_15(0);
}
void qpsi_baud_rate(uint32_t baud)
{
	uint32_t baud_div = 200000000 / baud;

	raw_writel(0xA1000008, 0);
	chip_hw_udelay(10);
	raw_writel(0xA1000014, baud_div & 0xFFFFU);
	raw_writel(0xA1000008, 1);
	chip_hw_udelay(10);

}
uint32_t true_data = 0;
uint32_t false_data = 0;

int32_t data_verify(uint8_t *w_buffer, uint8_t *r_buffer,uint32_t len)
{
	int32_t result = E_OK;
	uint32_t i = 0;
	true_data = 0;
	false_data = 0;

	for (i = 0; i < len; i++)
	{
		if (w_buffer[i] == r_buffer[i])
		{
			true_data++;
		}
		else {
			false_data++;
		}
	}

	if (true_data == len)
	{
		result = E_OK;
	}
	else {
		result = E_OBJ;
	}

	return result;
}
int32_t data_verify_word(uint32_t *w_buffer, uint32_t *r_buffer, uint32_t len)
{
	int32_t result = E_OK;
	uint32_t i = 0;
	true_data = 0;
	false_data = 0;

	for (i = 0; i < len; i++)
	{
		if (w_buffer[i] == r_buffer[i])
		{
			true_data++;
		}
		else {
			false_data++;
		}
	}

	if (true_data == len)
	{
		result = E_OK;
	}
	else {
		result = E_OBJ;
	}

	return result;
}
int32_t tm05_spi_case1(void *self, void *params, uint32_t len)
{

	uint32_t id = 0;
	uint32_t baudrate = 0;
	uint8_t type = 0;
	uint32_t data_len = 0;
	uint32_t addr = 0;
	int32_t result = E_OK;

do {
	auto_test_param_t *config = &spi_test_config;

	param_analy(params, len, config);

	id = config->id;
	baudrate = config->baudrate;
	type = config->type;
	data_len = config->len;
	addr = config->self_define;

	if (0 == id){
		/* SPIM Case*/
		IO_MUX_SPIM();
		chip_hw_udelay(10);
		switch (type)
		{
		case 0:
			result = spim_test_polling_case1(id, baudrate, read_buffer_w, data_len);
			break;
		case 1:
			data_buffer_install_word(write_buffer_w, data_len);
			result = spim_test_int_transfer_case2(id, baudrate, write_buffer_w, data_len);
			break;
		case 2:
			data_buffer_install_word(write_buffer_w, data_len);
			result = spim_test_int_receive_case2(id, baudrate, read_buffer_w, data_len);
			if (E_OK != result) {
				break;
			}
			chip_hw_mdelay(5000);
			result = data_verify_word(write_buffer_w, read_buffer_w, data_len);
			break;
		case 3:
			data_buffer_install_word(write_buffer_w, data_len);

			result = spim_test_DMA_write_case4(id, baudrate, write_buffer_w, data_len);
			break;
		case 4:
			data_buffer_install_word(write_buffer_w, data_len);
			result = spim_test_DMA_read_case4(id, baudrate, read_buffer_w, data_len);
			if (E_OK != result) {
				break;
			}
			result = data_verify_word(write_buffer_w, read_buffer_w, data_len);
			break;
		default:
			break;
		}
	} else if (1 == id) {
		/* SPIS Case*/
		IO_MUX_SPIS();
		chip_hw_udelay(10);
		switch (type)
		{
		case 0:
			data_buffer_install_word(write_buffer_w, data_len);
			result = spis_test_polling_read_case11(id, baudrate, read_buffer_w, data_len);
			if (E_OK != result) {
				break;
			}
			result = data_verify_word(write_buffer_w, read_buffer_w, data_len);
			break;
		case 1:
			data_buffer_install_word(write_buffer_w, data_len);
			result = spis_test_polling_write_case11(id, baudrate, write_buffer_w, data_len);
			break;
		case 2:
			data_buffer_install_word(write_buffer_w, data_len);
			result = spis_test_int_receive_case2(id, baudrate, read_buffer_w, data_len);
			if (E_OK != result) {
				break;
			}
			chip_hw_mdelay(5000);
			result = data_verify_word(write_buffer_w, read_buffer_w, data_len);
			break;
		case 3:
			data_buffer_install_word(write_buffer_w, data_len);
			result = spis_test_int_transfer_case2(id, baudrate, write_buffer_w, data_len);
			break;
		default:
			break;
		}
	} else if (2 == id) {
		/* QSPI Case*/
		result = flash_init();
		if (E_OK != result) {
			break;
		}

		/* config QSPI baud */
		//qpsi_baud_rate(baudrate);

		switch (type)
		{
		case 0:

			data_buffer_install_byte(write_buffer_b, data_len);

			result = spiq_test_polling_flash_case7(addr, write_buffer_b, read_buffer_b, data_len);

			chip_hw_mdelay(2);

			result = data_verify(write_buffer_b, read_buffer_b, data_len);

			break;
		case 1:

			data_buffer_install_byte(write_buffer_b, data_len);

			result = flash_memory_erase(addr, data_len);

			result = spiq_test_int_transfer_case10(write_buffer_b, data_len, addr);
			break;
		case 2:
			data_buffer_install_byte(write_buffer_b, data_len);

			result = spiq_test_int_receive_case10(read_buffer_b, data_len, addr);

			chip_hw_mdelay(5000);

			result = data_verify(write_buffer_b, read_buffer_b, data_len);
			break;
		case 3:
			data_len = 64;

			data_buffer_install_word(write_buffer_w, data_len);

			result = flash_memory_erase(addr, data_len);

			result = spiq_test_dma_write_case11(addr, write_buffer_w, data_len);
			break;
		case 4:
			data_len = 64;

			data_buffer_install_word(write_buffer_w, data_len);

			result = spiq_test_dma_read_case11(addr, read_buffer_w, data_len);

			result = data_verify_word(write_buffer_w, read_buffer_w, 64);
			break;
		default:
			break;
		}
	}

} while (0);

	return result;
}

//SPI_VALIDATION_SYS_CASE(SPI_CID_1, tm05_spi_case1);
VALIDATION_SYS_CASE_DEFINE(MODULE_SPI_ID, 1, tm05_spi_case1, NULL);
