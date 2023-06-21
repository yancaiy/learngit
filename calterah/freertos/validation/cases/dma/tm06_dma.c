#include "tm06_dma.h"
#include "validation.h"
#include "vmodule.h"
#include "dma_hal.h"
#include "dw_dmac.h"

#define SINGLE_MODE 								(1)
#define MULTI_MODE									(2)

#define BB_SHARED_MEM_ADDR1							(0x5003fffc)
#define BB_SHARED_MEM_ADDR2							(0x50049000)

#define SUPPORT_32BITS								(0x4)
#define SUPPORT_16BITS								(0x2)
#define SUPPORT_8BITS								(0x1)

/* BB 10 memory */
#define BB_MEM_ACT_COD								0
#define BB_MEM_ACT_SAM								1	// only read
#define BB_MEM_ACT_WIN								2
#define BB_MEM_ACT_NVE								3
#define BB_MEM_ACT_BUF								4
#define BB_MEM_ACT_COE								5
#define BB_MEM_ACT_MAC								6
#define BB_MEM_ACT_RLT								7	// Specific rules
#define BB_MEM_ACT_ANC								8
#define BB_MEM_ACT_DML								9
#define BB_MEM_ACT_TON								10
#define BB_MEM_ACT_STFT								11

#define BB_REG_SYS_MEM_ACT							(0x90000014)	// it's 0xC00014 on MP

#define BLOCK0_BB_VALID_SIZE						(0x200000)	/* 256Bytes */
#define BLOCK1_BB_VALID_SIZE						(0x200000)	/* 2M */
#define BLOCK2_BB_VALID_SIZE						(0x4800)	/* 18K */
#define BLOCK3_BB_VALID_SIZE						(0x2800)	/* 10K */
#define BLOCK4_BB_VALID_SIZE						(0x200000)	/* 2M */
#define BLOCK5_BB_VALID_SIZE						(0xE000)	/* 56K */
#define BLOCK6_BB_VALID_SIZE						(0x10000)	/* 64K */
#define BLOCK7_BB_VALID_SIZE						(0x2000)	/* 8K */
#define BLOCK8_BB_VALID_SIZE						(0x100)		/* 256Bytes */
#define BLOCK9_BB_VALID_SIZE						(0x8000)	/* 32K */


#define DMA_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_DMA_ID, cid, f_init, NULL)

#define DMA_MEMCOPY_DEFAULT_DESC(desc) do { \
		(desc)->transfer_type = MEM_TO_MEM; \
		(desc)->priority = DMA_CHN_PRIOR_MAX; \
		(desc)->src_desc.addr_mode = ADDRESS_INCREMENT; \
		(desc)->src_desc.hw_if = 0; \
		(desc)->src_desc.sts_upd_en = 0; \
		(desc)->src_desc.hs = HS_SELECT_SW; \
		(desc)->src_desc.sts_addr = 0; \
		(desc)->dst_desc.addr_mode = ADDRESS_INCREMENT; \
		(desc)->dst_desc.hw_if = 0; \
		(desc)->dst_desc.sts_upd_en = 0; \
		(desc)->dst_desc.hs = HS_SELECT_SW; \
		(desc)->dst_desc.sts_addr = 0;\
	} while(0)


static volatile uint32_t dma_done = 0;
extern dma_drv_t drv_dma;

dma_trans_desc_t global_desc = {
	.work_mode = DMA_SRC_NR_NG_NLLP_DST_NR_NS_NLLP,

	.src_tr_width = eTR_WIDTH_32_BITS,
	.src_desc = {
		.burst_tran_len = 2,			// 0-->len1, 1-->len4, 2-->len8
		.addr_mode = ADDRESS_INCREMENT,
		.gather_cnt = 4,				// data
		.gather_iv = 1,					// interval
	},

	.dst_tr_width = eTR_WIDTH_32_BITS,
	.dst_desc = {
		.burst_tran_len = 2,			// 0-->len1, 1-->len4, 2-->len8
		.addr_mode = ADDRESS_INCREMENT,
		.scatter_cnt = 2,				// data
		.scatter_iv = 2,				// interval
	}
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
			if (count++ >= len) {
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
						raw_writel(addr, j + data);
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


static void dma_memcopy_callback(void *params)
{
	int32_t result = dma_release_channel((uint32_t)params);
	if (E_OK != result) {
	}
}


static void dma_done_callback(void *params)
{
	int32_t result = dma_release_channel((uint32_t)params);
	if (E_OK != result) {
		EMBARC_PRINTF("DMA channel release failed.\r\n");
	} else {
		EMBARC_PRINTF("\nDMA Done\r\n\n");
		dma_done = 1;
	}
}


static int32_t dma_raw_memcopy(uint32_t *dst, uint32_t *src, uint32_t len, dma_chn_callback func, dma_trans_desc_t *desc_config)
{
	int32_t result = E_OK;

	uint32_t chn_id = 0;
	dma_trans_desc_t *desc = NULL;
	dma_trans_desc_t instance_desc;

	do {
		if ((NULL == dst) || (NULL == src) || (0 == len)) {
			result = E_PAR;
			break;
		}

		/* init the global drv_dma struct in dma_hal.c */
		DMA_MEMCOPY_DEFAULT_DESC(&instance_desc);
		desc = &instance_desc;

		drv_dma.dma_chn[chn_id].work_mode = desc_config->work_mode;

		desc->work_mode = desc_config->work_mode;
		desc->src_tr_width = desc_config->src_tr_width;
		desc->src_desc.burst_tran_len = desc_config->src_desc.burst_tran_len;	//msize
		desc->src_desc.gather_cnt = desc_config->src_desc.gather_cnt;
		desc->src_desc.gather_iv = desc_config->src_desc.gather_iv;
		desc->src_desc.addr = *src;

		desc->dst_tr_width = desc_config->dst_tr_width;
		desc->dst_desc.burst_tran_len = desc_config->dst_desc.burst_tran_len;	//msize
		desc->dst_desc.scatter_cnt = desc_config->dst_desc.scatter_cnt;
		desc->dst_desc.scatter_iv = desc_config->dst_desc.scatter_iv;
		desc->dst_desc.addr = *dst;

		desc->block_transfer_size = len;

		dma_transfer(desc, func);
#if 0
		result = dma_config_channel(chn_id, desc, 1);
		if (E_OK != result) {
			uint32_t cpu_status = arc_lock_save();
			result = dma_release_channel(chn_id);
			arc_unlock_restore(cpu_status);
			break;
		}

		if (NULL != func) {
			chn_isr_callback = func;
		}

		auto_reload_en = (((desc->work_mode >> 2) & 0x1) ? (1) : (0)) || \
						(((desc->work_mode >> 5) & 0x1) ? (1) : (0));
		if (auto_reload_en) {
			int_type = INT_BLOCK;
		}

		result = dma_start_channel(chn_id, int_type, chn_isr_callback);
		if (E_OK != result) {
			break;
		}
#endif
	} while (0);

	return result;
}


static uint32_t src_g_data_array[1024*10] = {0};
static uint32_t dst_g_data_array[1024*10] = {0};

auto_test_param_t dma_test_config;

static int32_t dma_compare_blockdata(uint32_t *dst_addr, uint32_t *src_addr, dma_trans_desc_t *desc, uint32_t block_cnt, uint32_t length)
{
	int32_t result = E_OK;

	uint32_t dst_data = 0, src_data = 0;
	uint32_t dst_scatter_en = 0, src_gather_en = 0;
	uint32_t idx = 0, checked_length = 0;

	src_gather_en = ((desc->work_mode >> 4) & 0x1) ? (1) : (0);
	dst_scatter_en = ((desc->work_mode >> 1) & 0x1) ? (1) : (0);

	xprintf("block_cnt: %d, start to compare data! \r\n", block_cnt);

	for (idx = 0; idx < DMAC_BLOCK_TS_MAX; idx++) {
		checked_length = (idx + 1) + (block_cnt * DMAC_BLOCK_TS_MAX);

		if (checked_length >= length) {
			break;
		}

		if (src_gather_en) {
			if ((idx != 0) && (!(idx % desc->src_desc.gather_cnt))) {
				*src_addr += desc->src_desc.gather_iv * (1 << desc->src_tr_width);
			}
		}

		if (dst_scatter_en) {
			if ((idx != 0) && (!(idx % desc->dst_desc.scatter_cnt))) {
				*dst_addr += desc->dst_desc.scatter_iv * (1 << desc->dst_tr_width);
			}
		}

		switch (desc->src_tr_width) {
			case eTR_WIDTH_8_BITS:
				src_data = raw_readb(*src_addr);
				break;
			case eTR_WIDTH_16_BITS:
				src_data = raw_readh(*src_addr);
				break;
			case eTR_WIDTH_32_BITS:
				src_data = raw_readl(*src_addr);
				break;
			default:
				xprintf("dma_chn->src_tr_width error! \r\n", idx+1);
				break;
		}

		switch (desc->dst_tr_width) {
				case eTR_WIDTH_8_BITS:
					dst_data = raw_readb(*dst_addr);
					break;
				case eTR_WIDTH_16_BITS:
					dst_data = raw_readh(*dst_addr);
					break;
				case eTR_WIDTH_32_BITS:
					dst_data = raw_readl(*dst_addr);
					break;
				default:
					xprintf("dma_chn->dst_tr_width error! \r\n", idx+1);
					break;
		}

		if (dst_data != src_data) {
			EMBARC_PRINTF("\n[%s] test result: Fail! \r\n", __func__);
			xprintf("block_cnt  : %d    index   : %d \r\n", block_cnt, idx+1);
			xprintf("src addr: 0x%x, src data: 0x%x \r\n", *src_addr, src_data);
			xprintf("dst addr: 0x%x, dst data: 0x%x \r\n", *dst_addr, dst_data);
			result = E_SYS;
		}else {
			//xprintf("#########	src addr: 0x%x, src data: 0x%x \r\n", *src_addr, src_data);
			//xprintf("#########	dst addr: 0x%x, dst data: 0x%x \r\n", *dst_addr, dst_data);
		}

		switch (desc->src_desc.addr_mode) {
			case ADDRESS_INCREMENT:
				*src_addr += (1 << desc->src_tr_width);
				break;
			case ADDRESS_DECREMENT:
				*src_addr -= (1 << desc->src_tr_width);
				break;
			case ADDRESS_FIXED:
				break;
			case ADDRESS_FIXED_1:
			default:
				/* nothing to do? */
				break;
		}

		switch (desc->dst_desc.addr_mode) {
			case ADDRESS_INCREMENT:
				*dst_addr += (1 << desc->dst_tr_width);
				break;
			case ADDRESS_DECREMENT:
				*dst_addr -= (1 << desc->dst_tr_width);
				break;
			case ADDRESS_FIXED:
				break;
			case ADDRESS_FIXED_1:
			default:
				/* nothing to do? */
				break;
		}
	}

	return result;
}


static int32_t dma_mem_check(uint32_t *dst, uint32_t *src, uint32_t length, dma_trans_desc_t *desc)
{
	int32_t result = E_OK;

	/* check the data in destination address */
	while (1) {
		if (dma_done) {
			uint32_t init_dst_addr = *dst, init_src_addr = *src;
			uint32_t dst_reload_en = 0, src_reload_en = 0;
			uint32_t dst_llp_en = 0, src_llp_en = 0;
			uint32_t cnt = 0;

			dst_llp_en = ((desc->work_mode >> 0) & 0x1) ? (1) : (0);
			src_llp_en = ((desc->work_mode >> 3) & 0x1) ? (1) : (0);

			dst_reload_en = ((desc->work_mode >> 2) & 0x1) ? (1) : (0);
			src_reload_en = ((desc->work_mode >> 5) & 0x1) ? (1) : (0);

			if ((!dst_llp_en) && (!src_llp_en) && (!dst_reload_en) && (!src_reload_en)) {
				if (length > DMAC_BLOCK_TS_MAX) {
					EMBARC_PRINTF("[single-block]setting lenght error\r\n");
					EMBARC_PRINTF("lenght ought to be less than DMAC_BLOCK_TS_MAX in single block mode!\r\n");
					result = E_PAR;
				}
			} else {
				if (length <= DMAC_BLOCK_TS_MAX) {
					EMBARC_PRINTF("[multi-block]setting lenght error\r\n");
					EMBARC_PRINTF("lenght ought to be more than DMAC_BLOCK_TS_MAX in multi block mode!\r\n");
					result = E_PAR;
				}
			}

			if (result == E_OK) {
				for (cnt = 0; cnt < drv_dma.total_block_cnt; cnt++) {
					if (dst_reload_en) {
						init_dst_addr = *dst ;
					}

					if (src_reload_en) {
						init_src_addr = *src;
					}

					result = dma_compare_blockdata(&init_dst_addr, &init_src_addr, desc, cnt, length);
				}
			}

			if (result < 0) {
				EMBARC_PRINTF("\nDMA test fail!!!, mode: 0x%x\r\n", desc->work_mode);
				break;
			} else {
				EMBARC_PRINTF("\nDMA test success!!!, mode: 0x%x\r\n", desc->work_mode);
			}

			/* verify finish! */
			dma_done = 0;
			chip_hw_mdelay(1000);
			break;
		} else {
			chip_hw_mdelay(100);
		}
	}

	return result;
}


/* Memory to Memory */
int32_t dma_mem2mem(uint32_t *dst, uint32_t *src, uint32_t length, dma_trans_desc_t *desc)
{
	int32_t result = E_OK;

	EMBARC_PRINTF("\r\n######################################!\r\n");
	EMBARC_PRINTF("###### DMA Test start! mode:0x%x #####\r\n", desc->work_mode);
	EMBARC_PRINTF("######################################!\r\n");
	EMBARC_PRINTF("init src addr:0x%x, dst addr:0x%x\r\n", *src, *dst);

	/* initialize the src memory to 0*/
	init_mem(*src, 0x0, length, desc->src_tr_width, 0);

	/* initialize the dst memory to 0*/
	init_mem(*dst, 0xff, length, desc->dst_tr_width, 0);

	/* filing the memory in source address */
	init_mem(*src, 0, 1024*5, desc->src_tr_width, 1);	//init 20k

	result = dma_raw_memcopy(dst, src, length, dma_done_callback, desc);
	if (E_OK != result) {
		EMBARC_PRINTF("DMA copy init failed%d\r\n", result);
	}

	result = dma_mem_check(dst, src, length, desc);
	if (E_OK != result) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
	}

	return result;
}


static int32_t dma_transfer_test(uint32_t *dst, uint32_t *src, uint32_t src_tr_width, uint32_t dst_tr_width, uint32_t length, uint32_t type)
{
	int32_t result = E_OK;

	if ((NULL == dst) || (NULL == src)) {
		result = E_PAR;
	}

	global_desc.src_tr_width = src_tr_width;
	global_desc.dst_tr_width = dst_tr_width;

	if (length <= DMAC_BLOCK_TS_MAX) {
		global_desc.work_mode = type;
		result = dma_mem2mem(dst, src, length, &global_desc);
		if (result != E_OK) {
			EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
			return result;
		}
	} else {
		global_desc.work_mode = type;
		result = dma_mem2mem(dst, src, length, &global_desc);
		if (result != E_OK) {
			EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
			return result;
		}
	}

	EMBARC_PRINTF("######################################!\r\n");
	EMBARC_PRINTF("##############src_tr_width: 0x%x, dst_tr_width: 0x%x, DMA Test end!###########\r\n", src_tr_width, dst_tr_width);
	EMBARC_PRINTF("######################################!\r\n");

	return result;
}


static int32_t dma_cpumem2cpumem(uint32_t *dst, uint32_t *src, uint32_t transfer_type, uint32_t length, uint32_t access, uint32_t type)
{
	int32_t result = E_OK;

	if (transfer_type == SINGLE_MODE) {
		if (length > DMAC_BLOCK_TS_MAX){
			result = E_PAR;
			return result;
		} 
	}else if (transfer_type == MULTI_MODE) {
		if (length <= DMAC_BLOCK_TS_MAX){
			result = E_PAR;
			return result;
		} 
	} else {
		result = E_PAR;
	}

	if (access&SUPPORT_32BITS) {
		result = dma_transfer_test(dst, src, eTR_WIDTH_32_BITS, eTR_WIDTH_32_BITS, length, type);
		if (result != E_OK) {
			EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
			return result;
		}
	}

	if (access&SUPPORT_16BITS) {
		result = dma_transfer_test(dst, src, eTR_WIDTH_16_BITS, eTR_WIDTH_16_BITS, length, type);
		if (result != E_OK) {
			EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
			return result;
		}
	}

	if (access&SUPPORT_8BITS) {
		result =dma_transfer_test(dst, src, eTR_WIDTH_8_BITS, eTR_WIDTH_8_BITS, length, type);
		if (result != E_OK) {
			EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
			return result;
		}
	}

	return result;
}

#if 0

/*TBD: waiting for bb handshake ready */
static int32_t dma_p2m_bb2cpu(uint32_t bb_active_mem_block, uint32_t *dst, uint32_t *src, uint32_t length, uint8_t hw_if, enum dma_transfer_type transfer_type)
{
	int32_t result = E_OK;

	bb_enable(1);

	switch (bb_active_mem_block) {
		case BB_MEM_ACT_COD:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_COD);
			EMBARC_PRINTF("------BB_MEM_ACT_COD------\r\n");
			break;
		case BB_MEM_ACT_WIN:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_WIN);
			EMBARC_PRINTF("------BB_MEM_ACT_WIN------\r\n");
			break;
		case BB_MEM_ACT_NVE:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_NVE);
			EMBARC_PRINTF("------BB_MEM_ACT_NVE------\r\n");
			break;
		case BB_MEM_ACT_BUF:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_BUF);
			EMBARC_PRINTF("------BB_MEM_ACT_BUF------\r\n");
			break;
		case BB_MEM_ACT_COE:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_COE);
			EMBARC_PRINTF("------BB_MEM_ACT_COE------\r\n");
			break;
		case BB_MEM_ACT_MAC:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_MAC);
			EMBARC_PRINTF("------BB_MEM_ACT_MAC------\r\n");
			break;
		case BB_MEM_ACT_RLT:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_RLT);
			EMBARC_PRINTF("------BB_MEM_ACT_RLT------\r\n");
			break;
		case BB_MEM_ACT_ANC:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_ANC);
			EMBARC_PRINTF("------BB_MEM_ACT_ANC------\r\n");
			break;
		case BB_MEM_ACT_DML:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_DML);
			EMBARC_PRINTF("------BB_MEM_ACT_DML------\r\n");
			break;
		case BB_MEM_ACT_TON:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_TON);
			EMBARC_PRINTF("------BB_MEM_ACT_TON------\r\n");
			break;
		case BB_MEM_ACT_STFT:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_STFT);
			EMBARC_PRINTF("------BB_MEM_ACT_STFT------\r\n");
			break;
		default:
			EMBARC_PRINTF("[%s]line:%d, parameter error, %d!\r\n", __func__, __LINE__, result);
			result = E_PAR;
	}

	if (result == E_OK) {
		dma_trans_desc_t desc;

		desc.transfer_type = transfer_type;
		desc.priority = DMA_CHN_PRIOR_3;
		desc.block_transfer_size = length;

		desc.work_mode = DMA_SRC_NR_NG_NLLP_DST_NR_NS_NLLP;

		desc.src_desc.burst_tran_len = BURST_LEN8;
		desc.src_desc.addr_mode = ADDRESS_INCREMENT;
		desc.src_desc.sts_upd_en = 0;
		desc.src_desc.hs = HS_SELECT_SW;
		desc.src_desc.addr = (uint32_t)(*src);
		desc.src_tr_width = eTR_WIDTH_32_BITS;

		desc.dst_desc.burst_tran_len = BURST_LEN8;
		desc.dst_desc.addr_mode = ADDRESS_FIXED;
		desc.dst_desc.hw_if = hw_if;
		desc.dst_desc.sts_upd_en = 0;
		desc.dst_desc.hs = HS_SELECT_HW;
		desc.dst_desc.addr = (uint32_t)(*dst);
		desc.dst_tr_width = eTR_WIDTH_32_BITS;

		dma_transfer(&desc, dma_done_callback);

		result = dma_mem_check(dst, src, length, &desc);
		if (E_OK != result) {
			EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		}
	}

	return result;
}


static int32_t dma_cpumem2bb_bb2cpumem(uint32_t bb_active_mem_block, uint32_t *dst, uint32_t *src, uint32_t transfer_type, uint32_t length, uint32_t access)
{
	int32_t result = E_OK;

	bb_enable(1);

	switch (bb_active_mem_block) {
		case BB_MEM_ACT_COD:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_COD);
			EMBARC_PRINTF("------BB_MEM_ACT_COD------\r\n");
			break;
		case BB_MEM_ACT_WIN:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_WIN);
			EMBARC_PRINTF("------BB_MEM_ACT_WIN------\r\n");
			break;
		case BB_MEM_ACT_NVE:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_NVE);
			EMBARC_PRINTF("------BB_MEM_ACT_NVE------\r\n");
			break;
		case BB_MEM_ACT_BUF:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_BUF);
			EMBARC_PRINTF("------BB_MEM_ACT_BUF------\r\n");
			break;
		case BB_MEM_ACT_COE:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_COE);
			EMBARC_PRINTF("------BB_MEM_ACT_COE------\r\n");
			break;
		case BB_MEM_ACT_MAC:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_MAC);
			EMBARC_PRINTF("------BB_MEM_ACT_MAC------\r\n");
			break;
		case BB_MEM_ACT_RLT:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_RLT);
			EMBARC_PRINTF("------BB_MEM_ACT_RLT------\r\n");
			break;
		case BB_MEM_ACT_ANC:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_ANC);
			EMBARC_PRINTF("------BB_MEM_ACT_ANC------\r\n");
			break;
		case BB_MEM_ACT_DML:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_DML);
			EMBARC_PRINTF("------BB_MEM_ACT_DML------\r\n");
			break;
		case BB_MEM_ACT_TON:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_TON);
			EMBARC_PRINTF("------BB_MEM_ACT_TON------\r\n");
			break;
		case BB_MEM_ACT_STFT:
			raw_writel(BB_REG_SYS_MEM_ACT, BB_MEM_ACT_STFT);
			EMBARC_PRINTF("------BB_MEM_ACT_STFT------\r\n");
			break;
		default:
			EMBARC_PRINTF("[%s]line:%d, parameter error, %d!\r\n", __func__, __LINE__, result);
			result = E_PAR;
	}

	if (result == E_OK) {
		result = dma_cpumem2cpumem(dst, src, transfer_type, length, access);
		if (result != E_OK) {
			EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
			return result;
		}
	}

	return result;
}

static int32_t dma_cpumem2bb_nosharedmem(uint32_t *dst, uint32_t *src, uint32_t transfer_type, uint32_t length, uint32_t access)
{
	int32_t result =E_OK;

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_COD, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_WIN, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_NVE, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_BUF, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_COE, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_MAC, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	//result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_RLT, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_ANC, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	//result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_DML, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_TON, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2bb_bb2cpumem(BB_MEM_ACT_STFT, dst, src, transfer_type, length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	return result;
}


int32_t dma_mem2mem_bb_nosharedmem_case(void *self, void *params, uint32_t psize)
{
	/* BB non-shared memory test*/
	int32_t result = E_OK;

	uint32_t single_length = 1023;
	uint32_t multi_length = 5 * 1024;

	/* BB Memory wrod only */
	uint32_t access = SUPPORT_32BITS;

	uint32_t cpu2bb_dst = (uint32_t)(REL_REGBASE_BB_MEM);
	uint32_t cpu2bb_src = (uint32_t)src_g_data_array;

	uint32_t bb2cpu_dst = (uint32_t)dst_g_data_array;
	uint32_t bb2cpu_src = (uint32_t)(REL_REGBASE_BB_MEM);

	uint32_t *payload = (uint32_t *)params;
	if (payload != NULL) {
		/* source and destination */
		if (payload[0] != 0 ){
			cpu2bb_dst = payload[0];
		}

		if (payload[1] != 0 ){
			cpu2bb_src = payload[1];
		}

		if (payload[2] != 0 ){
			cpu2bb_dst = payload[2];
		}

		if (payload[3] != 0 ){
			bb2cpu_src = payload[3];
		}

		if (payload[4] != 0) {
			single_length = payload[4];
		}

		if (payload[5] != 0) {
			multi_length = payload[5];
		}
	}

	validation_case_t *case_ptr = (validation_case_t *)self;
	EMBARC_PRINTF("Module id:%d, Case id:%d \r\n", case_ptr->mod_id, case_ptr->case_id);

	dma_cpumem2bb_nosharedmem(&cpu2bb_dst, &cpu2bb_src, SINGLE_MODE, single_length, access);

	dma_cpumem2bb_nosharedmem(&bb2cpu_dst, &bb2cpu_src, SINGLE_MODE, multi_length, access);

	return result;
}


int32_t dma_mem2mem_bb_sharedmem_case(void *self, void *params, uint32_t psize)
{
	/* BB shared memory test */
	int32_t result = E_OK;

	uint32_t single_length = 1023;
	uint32_t multi_length = 5 * 1024;

	/* BB shared Memory wrod only */
	uint32_t access = SUPPORT_32BITS;
	uint32_t cpu2cpu_dst = (uint32_t)dst_g_data_array;
	uint32_t cpu2cpu_src = (uint32_t)src_g_data_array;

	uint32_t cpu2bb_dst = (uint32_t)(BB_SHARED_MEM_ADDR1);
	uint32_t cpu2bb_src = (uint32_t)src_g_data_array;

	uint32_t bb2cpu_dst = (uint32_t)dst_g_data_array;
	uint32_t bb2cpu_src = (uint32_t)(BB_SHARED_MEM_ADDR1);

	uint32_t bb2bb_dst = (uint32_t)(BB_SHARED_MEM_ADDR2);
	uint32_t bb2bb_src = (uint32_t)(BB_SHARED_MEM_ADDR1);

	uint32_t *payload = (uint32_t *)params;

	if (payload != NULL) {
		/* source and destination */
		if (payload[0] != 0 ){
			cpu2cpu_dst = payload[0];
		}

		if (payload[1] != 0 ){
			cpu2cpu_src = payload[1];
		}

		if (payload[2] != 0 ){
			cpu2bb_dst = payload[2];
		}

		if (payload[3] != 0 ){
			cpu2bb_src = payload[3];
		}

		if (payload[4] != 0 ){
			bb2cpu_dst = payload[4];
		}

		if (payload[5] != 0 ){
			bb2cpu_src = payload[5];
		}

		if (payload[6] != 0 ){
			bb2bb_dst = payload[6];
		}

		if (payload[7] != 0 ){
			bb2bb_src = payload[7];
		}

		if (payload[8] != 0) {
			single_length = payload[8];
		}

		if (payload[9] != 0) {
			multi_length = payload[9];
		}
	}

	validation_case_t *case_ptr = (validation_case_t *)self;
	EMBARC_PRINTF("Module id:%d, Case id:%d \r\n", case_ptr->mod_id, case_ptr->case_id);

	/* CPU Memory to CPU memory */
	result = dma_cpumem2cpumem(&cpu2cpu_dst, &cpu2cpu_src, SINGLE_MODE, single_length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2cpumem(&cpu2cpu_dst, &cpu2cpu_src, MULTI_MODE, single_length, access);
	if (result != E_OK) {
		return result;
	}


	/* CPU Memory to Baseband memory. */
	result = dma_cpumem2cpumem(&cpu2bb_dst, &cpu2bb_src, SINGLE_MODE, single_length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2cpumem(&cpu2bb_dst, &cpu2bb_src, MULTI_MODE, multi_length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}


	/* Baseband Memory to CPU memory. */
	result = dma_cpumem2cpumem(&bb2cpu_dst, &bb2cpu_src, SINGLE_MODE, single_length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2cpumem(&bb2cpu_dst, &bb2cpu_src, MULTI_MODE, multi_length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}


	/* Baseband Memory to Baseband Memory. */
	result = dma_cpumem2cpumem(&bb2bb_dst, &bb2bb_src, SINGLE_MODE, single_length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	result = dma_cpumem2cpumem(&bb2bb_dst, &bb2bb_src, MULTI_MODE, multi_length, access);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}
	

	return result;
}
#endif


int32_t dma_mem2mem_cpu_mem_case(void *self, void *params, uint32_t psize)
{
	uint32_t id = 0;
	uint32_t baudrate = 0;
	uint8_t type = 0;
	uint32_t data_len = 0;
	int32_t result = E_OK;

	auto_test_param_t *config = &dma_test_config;

	param_analy(params, psize, config);

	id = config->id;
	baudrate = config->baudrate;
	type = config->type;
	data_len = config->len;

	uint32_t dst = 0, src = 0;
	uint32_t length = data_len;

	uint32_t access = baudrate;

	src = (uint32_t)src_g_data_array;
	dst = (uint32_t)dst_g_data_array;

	validation_case_t *case_ptr = (validation_case_t *)self;
	EMBARC_PRINTF("Module id:%d, Case id:%d \r\n", case_ptr->mod_id, case_ptr->case_id);

	//id:2 MULTI_MODE/1 SINGLE_MODE
	result = dma_cpumem2cpumem(&dst, &src, id, length, access, type);
	if (result != E_OK) {
		EMBARC_PRINTF("[%s][line:%d] error : %d \r\n", __func__, __LINE__, result);
		return result;
	}

	return result;
}

VALIDATION_SYS_CASE_DEFINE(MODULE_DMA_ID, 1, dma_mem2mem_cpu_mem_case, NULL);
