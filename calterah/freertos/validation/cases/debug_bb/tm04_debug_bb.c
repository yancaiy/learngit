#include "tm04_debug_bb.h"
#include "validation.h"
#include "vmodule.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "debug.h"
#include "debug_reg.h"
#include "debug_hal.h"
#include "dma_hal.h"
#include "arc_cache.h"

#define DEBUG_BB_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_DEBUG_BB_ID, cid, f_init, NULL)

#define LEN_WORD							(64)

static uint32_t src_g_data_array[LEN_WORD] = {0};
static uint32_t dst_g_data_array[LEN_WORD] = {0};

volatile uint32_t err_cnt = 0;
volatile uint32_t total_time = 0;
volatile bool loopback_stress = false;

static volatile uint32_t dma_rx_done = 0;
static volatile uint32_t dma_tx_done = 0;

static lvds_tx_config_t lvds_tx_config = {
	.lvds_src = eDBG_LVDS_APB_BUS,
	.payload_mode = eDBG_PL_ONE_PACKAGE_256BYTES,
	.tx_mode = eDBG_STANDARD_MODE,
	.header = 0xc9,
	.vec_mode = eDBG_VEC_1AFF
};

static lvds_rx_config_t lvds_rx_config = {
	.payload_mode = eDBG_PL_ONE_PACKAGE_256BYTES,
	.infterface = eDBG_INTF_LVDS,
	.unit = eDBG_LVDS,
	.header = 0xc9,
	.hil_src = eHIL_LVDS
};

static dbgbus_tx_config_t dbgbus_tx_config = {
	.mode = eDBG_8BIT_MODE,
	.dbgbus_src = eDBG_DBGBUS_APB_BUS
};

static dbgbus_rx_config_t dbgbus_rx_config = {
	.mode = eDBG_8BIT_MODE,
	.hil_src = eHIL_DBGBUS
};


/* len unit is by word */
void init_mem(uint32_t addr, uint32_t data, uint32_t len, uint32_t type, uint32_t flag)
{
	uint32_t i = 0, j = 0;
	uint32_t count = 1;
	uint32_t max_uint = 0;

	switch (type){
		case eTR_WIDTH_8_BITS:
			max_uint = 0xFF;
			break;
		case eTR_WIDTH_16_BITS:
			max_uint = 0xFFFF;
			break;
		case eTR_WIDTH_32_BITS:
			max_uint = 0xFFFFFFFF;
			break;
		default:
			break;
	}

	for (i = 1; i <= (len / max_uint + 1); i++) {
		for (j = 1; j <= max_uint; j++) {
			if (count++ > len) {
				break;
			}

			if (flag == 1) {
				switch (type){
					case eTR_WIDTH_8_BITS:
						raw_writeb(addr, j + data);
						break;
					case eTR_WIDTH_16_BITS:
						raw_writeh(addr, j + data);
						break;
					case eTR_WIDTH_32_BITS:
						raw_writel(addr, j | (data << 24));
						break;
					default:
						break;
				}
			} else {
				switch (type){
					case eTR_WIDTH_8_BITS:
						raw_writeb(addr, data);
						break;
					case eTR_WIDTH_16_BITS:
						raw_writeh(addr, data);
						break;
					case eTR_WIDTH_32_BITS:
						raw_writel(addr, data);
						break;
					default:
						break;
				}
			}

			addr += (1 << type);
		}
	}
}


static void check_data(uint32_t *src, uint32_t *dst, uint32_t len)
{
	for(uint32_t i = 0; i < len; i++) {
		if (dst[i] != src[i]) {
			err_cnt++;
			EMBARC_PRINTF("src[%d]: %d, dst[%d]:%d\r\n", i, src[i], i, dst[i]);
			chip_hw_mdelay(1);
		}
	}
}


static void debug_dma_tx_done_callback(void *params)
{
	dma_tx_done = 1;
	EMBARC_PRINTF("debug_dma_tx_done \r\n");
	int32_t result = dma_release_channel((uint32_t)params);
	if (E_OK != result) {
		EMBARC_PRINTF("DMA channel release failed.\r\n");
	}
}


static void debug_dma_rx_done_callback(void *params)
{
	dma_rx_done = 1;
	EMBARC_PRINTF("debug_dma_rx_done\r\n");
	int32_t result = dma_release_channel((uint32_t)params);
	if (E_OK != result) {
		EMBARC_PRINTF("DMA channel release failed.\r\n");
	}
}


/*********************************************************************************/
/* DBGBUS send API and loopback test case */
/*********************************************************************************/
int32_t debug_apbbus2dbgbus(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	EMBARC_PRINTF("%s entry! \r\n", __func__);
	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);

	/* configuration */
	debug_dbgbus_datadump(&dbgbus_tx_config);

	/* filling data to the fifo of the debug module*/
	debug_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD);

	EMBARC_PRINTF("%s exit! \r\n", __func__);

	return result;
}


int32_t debug_apbbus2dbgbus_dma(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	dcache_flush();

	/* configuration */
	debug_dbgbus_datadump(&dbgbus_tx_config);

	/* filling data to the fifo of the debug module*/
	debug_dma_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD, debug_dma_tx_done_callback);

	return result;
}


int32_t debug_dbgbus_dma_loopback(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	//EMBARC_PRINTF("%s entry! \r\n", __func__);
	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	init_mem((uint32_t)dst_g_data_array, 1, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	dcache_flush();

	/* configuration */
	/* dbgbus tx */
	debug_dbgbus_datadump(&dbgbus_tx_config);
	/* dbgbus rx */
	debug_dbgbus_datahil(&dbgbus_rx_config);

	/* filling data to the fifo of the debug module*/
	debug_dma_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD, debug_dma_tx_done_callback);

	/* get data from the fifo of the debug module*/
	debug_dma_read(dst_g_data_array, LEN_WORD, debug_dma_rx_done_callback);

	while (1) {
		if ((dma_tx_done == 1) && (dma_rx_done == 1)) {
			dma_tx_done = 0;
			dma_rx_done = 0;
			break;
		} else {
			chip_hw_udelay(200);
		}
	}

	icache_invalidate();
	dcache_invalidate();

	check_data(src_g_data_array, dst_g_data_array, LEN_WORD);

	total_time++;

	EMBARC_PRINTF("%s exit!  total times:%d err_cnt:%d\r\n", __func__, total_time, err_cnt);

	if (loopback_stress == false) {
		total_time = 0;
		err_cnt = 0;
	}

	return result;
}


int32_t debug_dbgbus_loopback(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	//EMBARC_PRINTF("%s entry! \r\n", __func__);
	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	chip_hw_udelay(100);
	init_mem((uint32_t)dst_g_data_array, 1, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	chip_hw_udelay(100);

	/* configuration */
	/* dbgbus tx */
	debug_dbgbus_datadump(&dbgbus_tx_config);
	/* dbgbus rx */
	debug_dbgbus_datahil(&dbgbus_rx_config);

	/* filling data to the fifo of the debug module */
	debug_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD);


	/* get data from the fifo of the debug module */
	debug_read(dst_g_data_array, LEN_WORD);
	chip_hw_udelay(100);

	dcache_flush();

	dcache_invalidate();

	check_data(src_g_data_array, dst_g_data_array, LEN_WORD);

	total_time++;

	EMBARC_PRINTF("%s exit!  total times:%d err_cnt:%d\r\n", __func__, total_time, err_cnt);

	if (loopback_stress == false) {
		total_time = 0;
		err_cnt = 0;
	}

	return result;
}


/*********************************************************************************/
/* LVDS send API , loopback test case and header test case */
/*********************************************************************************/
int32_t debug_apbbus2lvds(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	EMBARC_PRINTF("%s entry! \r\n", __func__);
	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);

	/* configuration */
	debug_lvds_datadump(&lvds_tx_config);

	/* filling data to the fifo of the debug module*/
	debug_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD);

	EMBARC_PRINTF("%s exit! \r\n", __func__);
	return result;
}


int32_t debug_apbbus2lvds_dma(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_8_BITS, 1);

	/* configuration */
	debug_lvds_datadump(&lvds_tx_config);

	/* filling data to the fifo of the debug module*/
	debug_dma_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD, debug_dma_tx_done_callback);

	return result;
}


int32_t debug_lvds_dma_loopback(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	//EMBARC_PRINTF("%s entry! \r\n", __func__);
	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	chip_hw_udelay(100);
	init_mem((uint32_t)dst_g_data_array, 1, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	chip_hw_udelay(500);

	dcache_flush();

	//dcache_invalidate();
	//dcache_invalidate_mlines((uint32_t)dst_g_data_array,sizeof(dst_g_data_array));

	/* configuration */
	/* lvds tx */
	debug_lvds_datadump(&lvds_tx_config);
	/* lvds rx */
	debug_lvds_datahil(&lvds_rx_config);

	/* filling data to the fifo of the debug module*/
	debug_dma_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD, debug_dma_tx_done_callback);

	/* get data from the fifo of the debug module*/
	debug_dma_read(dst_g_data_array, LEN_WORD, debug_dma_rx_done_callback);

	while (1) {
		if ((dma_tx_done == 1) && (dma_rx_done == 1)) {
			dma_tx_done = 0;
			dma_rx_done = 0;
			break;
		} else {
			chip_hw_udelay(200);
		}
	}

	icache_invalidate();
	dcache_invalidate();

	check_data(src_g_data_array, dst_g_data_array, LEN_WORD);

	total_time++;

	EMBARC_PRINTF("%s exit!  total times:%d err_cnt:%d\r\n", __func__, total_time, err_cnt);

	if (loopback_stress == false) {
		total_time = 0;
		err_cnt = 0;
	}

	return result;
}


int32_t debug_lvds_loopback(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	//EMBARC_PRINTF("%s entry! \r\n", __func__);
	init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);
	init_mem((uint32_t)dst_g_data_array, 1, LEN_WORD, eTR_WIDTH_32_BITS, 1);

	/* configuration */
	/* lvds tx */
	debug_lvds_datadump(&lvds_tx_config);
	/* lvds rx */
	debug_lvds_datahil(&lvds_rx_config);

	/* filling data to the fifo of the debug module */
	debug_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD);

	/* get data from the fifo of the debug module */
	debug_read(dst_g_data_array, 64);

	chip_hw_udelay(100);

	//dcache_flush();

	//dcache_invalidate();

	check_data(src_g_data_array, dst_g_data_array, LEN_WORD);

	total_time++;

	EMBARC_PRINTF("%s exit!  total times:%d err_cnt:%d\r\n", __func__, total_time, err_cnt);

	if (loopback_stress == false) {
		total_time = 0;
		err_cnt = 0;
	}

	return result;
}


int32_t debug_lvds_header_test(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	for (uint32_t i = 0; i < 5; i++) {
		/* configuration */
		/* lvds tx */
		debug_lvds_datadump(&lvds_tx_config);
		/* lvds rx */
		debug_lvds_datahil(&lvds_rx_config);

		switch (i) {
			case 0:
				lvds_tx_config.header = 0xd7;
				lvds_rx_config.header = 0xd7;
				break;
			case 1:
				lvds_tx_config.header = 0xa8;
				lvds_rx_config.header = 0xa8;
				break;
			case 2:
				lvds_tx_config.header = 0xb5;
				lvds_rx_config.header = 0xb5;
				break;
			case 3:
				lvds_tx_config.header = 0xc9;
				lvds_rx_config.header = 0xc9;
				break;
			case 4:
				lvds_tx_config.header = 0x7b;
				lvds_rx_config.header = 0x7b;
				break;
		}

		loopback_stress = true;
		for (uint32_t j = 0; j < 1000; j++) {
	//		EMBARC_PRINTF("%s entry! \r\n", __func__);
			init_mem((uint32_t)src_g_data_array, 0, LEN_WORD, eTR_WIDTH_32_BITS, 1);
			init_mem((uint32_t)dst_g_data_array, 1, LEN_WORD, eTR_WIDTH_32_BITS, 1);
			dcache_flush();

			EMBARC_PRINTF("lvds header: 0x%x \r\n", lvds_rx_config.header);
			/* filling data to the fifo of the debug module */
			debug_write(eDBG_DUMP_DAT_OUT, src_g_data_array, LEN_WORD);

			/* get data from the fifo of the debug module */
			debug_read(dst_g_data_array, LEN_WORD);

			chip_hw_udelay(100);

			dcache_flush();

			dcache_invalidate();

			check_data(src_g_data_array, dst_g_data_array, LEN_WORD);

			total_time++;

			EMBARC_PRINTF("%s exit!  total times:%d err_cnt:%d\r\n", __func__, total_time, err_cnt);

			if (loopback_stress == false) {
				total_time = 0;
				err_cnt = 0;
			}
		}
		loopback_stress = false;
	}

	return result;
}


/* dump data from apb bus to debug bus */
DEBUG_BB_VALIDATION_SYS_CASE(DEBUG_BB_CID_0, debug_apbbus2dbgbus);
