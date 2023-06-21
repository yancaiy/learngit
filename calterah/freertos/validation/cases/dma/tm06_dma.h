#ifndef TM06_DMA_H
#define TM06_DMA_H

#include <string.h>
#include "embARC_debug.h"
#include "embARC.h"
#include "dma_hal.h"

/**
 * \name XIP validation test ID names
 * @{
 */
/* CPU Memory test */
#define DMA_CID_0			0

/* BB shared Memory test */
#define DMA_CID_1			1

/* BB no shared Memory test */
#define DMA_CID_2			2

typedef struct dma_chn_info{
	uint32_t work_mode;

	uint32_t gather_iv;
	uint32_t gather_cnt;
	uint32_t scatter_iv;
	uint32_t scatter_cnt;

	enum dma_address_mode src_addr_mode;
	enum dma_address_mode dst_addr_mode;

	uint32_t src_tr_width;
	uint32_t dst_tr_width;

	enum dma_burst_length src_burst_tran_len;
	enum dma_burst_length dst_burst_tran_len;
} dma_chn_info_t;

//int32_t dma_mem2mem(void *self, void *params, uint32_t psize);
int32_t dma_mem2mem_all_case(void *self, void *params, uint32_t psize);

#endif
