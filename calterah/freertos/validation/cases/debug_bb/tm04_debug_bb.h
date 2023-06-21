#ifndef TM04_DEBUG_BB_H
#define TM04_DEBUG_BB_H

#include <string.h>
#include "embARC_debug.h"
#include "embARC.h"

/* BB to LVDS in direct mode */
#define DEBUG_BB_CID_0			0

/* BB to LVDS in direct mode */
#define DEBUG_BB_CID_1			1

/* LVDS to BB in direct mode */
#define DEBUG_BB_CID_2			2

/* Memory to LVDS in DMA mode */
#define DEBUG_BB_CID_3			3

/* LVDS to Memory in DMA mode */
#define DEBUG_BB_CID_4			4

/* BB to DEBUG_BUS in direct mode */
#define DEBUG_BB_CID_5			5

/* DEBUG_BUS to BB in direct mode */
#define DEBUG_BB_CID_6			6

/* Memory to DEBUG_BUS in DMA mode */
#define DEBUG_BB_CID_7			7

/* DEBUG_BUS to Memory in DMA mode */
#define DEBUG_BB_CID_8			8

/* BB_IRQ[0] to BB request DMA */
#define DEBUG_BB_CID_9			9

/* BB_IRQ[1] to BB request DMA */
#define DEBUG_BB_CID_10			10

/* BB_IRQ[2] to BB request DMA */
#define DEBUG_BB_CID_11			11


int32_t debug_apbbus2lvds(void *self, void *params, uint32_t psize);
int32_t debug_apbbus2lvds_dma(void *self, void *params, uint32_t psize);
int32_t debug_lvds_header_test(void *self, void *params, uint32_t psize);

int32_t debug_lvds_dma_loopback(void *self, void *params, uint32_t psize);
int32_t debug_lvds_loopback(void *self, void *params, uint32_t psize);

int32_t debug_apbbus2dbgbus(void *self, void *params, uint32_t psize);
int32_t debug_apbbus2dbgbus_dma(void *self, void *params, uint32_t psize);

int32_t debug_dbgbus_loopback(void *self, void *params, uint32_t psize);
int32_t debug_dbgbus_dma_loopback(void *self, void *params, uint32_t psize);
#endif
