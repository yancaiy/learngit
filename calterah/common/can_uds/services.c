#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "flash.h"


typedef struct {
	uint8_t state;
       uint32_t flash_base;
	uint32_t mem_base;
	uint32_t total_size;
	uint32_t received_size;
} can_uds_download_t;

#define CAN_UDS_XFER_BLOCK_LEN		(0x100 + 2)
static can_uds_download_t uds_download;
void set_flash_addr(uint32_t addr)
{
	uds_download.flash_base = addr;
}

static uint32_t busdata2word(uint8_t *data, uint8_t len)
{
	uint32_t word = 0;
	while (len) {
		word |= (*data++ << ((len - 1) << 3));
		len -= 1;
	}
	return word;
}

void can_uds_request_download(pdu_t *rxdata, pdu_t *txdata, uint32_t tx_pdu_id)
{
	uint8_t rsp_code = 0xFF;
	//uint8_t dfi = 0;
	uint8_t alfid = 0;
	uint8_t mem_addr_len, mem_size_len;
	uint32_t mem_addr, mem_size;

	do {
		if ((NULL == rxdata) || (NULL == txdata)) {
			/* what happened? */
			break;
		}

		//dfi = rxdata->data[1];
		alfid = rxdata->data[2];
		mem_addr_len = alfid & 0xF;
		mem_size_len = (alfid >> 4) & 0xF;

		if (mem_addr_len + mem_size_len + 3 < rxdata->length) {
			/* incorrectMessageLengthOrInvalidFormat */
			rsp_code = 0x13;
			break;
		}

		if ((mem_addr_len > 4) || (mem_size_len > 4)) {
			/* Error: return NRC. */
			rsp_code = 0x31;
			break;
		}

		mem_addr = busdata2word(&rxdata->data[3], mem_addr_len);
		mem_size = busdata2word(&rxdata->data[3 + mem_addr_len], mem_size_len);
		if ((mem_addr < 0x770000) || (mem_addr + mem_size >= 0x7f0000)) {
			/* Error: return NRC. */
			rsp_code = 0x31;
		}

		uds_download.mem_base = mem_addr;
		uds_download.total_size = mem_size;
		uds_download.received_size = 0;
		uds_download.state = 1;

		rsp_code = 0;

		/* filling response. */
		txdata->data[0] = 0x74;
		txdata->data[1] = 3;
		txdata->data[2] = (CAN_UDS_XFER_BLOCK_LEN >> 16) & 0xFF;
		txdata->data[3] = (CAN_UDS_XFER_BLOCK_LEN >> 8) & 0xFF;
		txdata->data[4] = CAN_UDS_XFER_BLOCK_LEN & 0xFF;
		txdata->length = 5;

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		txdata->data[0] = 0x7F;
		txdata->data[1] = 0x34;
		txdata->data[2] = rsp_code;
		txdata->length = 3;
	}
}

static void trace_info(uint32_t value)
{
	static uint32_t idx = 0;
	raw_writel(0x7d0000 + (idx++ << 2), value);
}
void can_uds_transfer_data(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id)
{
	uint8_t rsp_code = 0xFF;

	static uint8_t bsc = 1;

	uint32_t cur_xfer_size, remain_size;
       /* like sequence number */
	uint8_t cur_bsc = bsc;
	uint8_t *dbuf = NULL;

	do {
		if ((NULL == rxdata) || (NULL == txdata)) {
			break;
		}

		remain_size = uds_download.total_size - uds_download.received_size;
		if (remain_size < CAN_UDS_XFER_BLOCK_LEN - 2) {
			cur_xfer_size = remain_size;
		} else {
			cur_xfer_size = CAN_UDS_XFER_BLOCK_LEN - 2;
		}

		if ((rxdata->length > CAN_UDS_XFER_BLOCK_LEN) || \
		    ((rxdata->length < CAN_UDS_XFER_BLOCK_LEN) && \
		     (rxdata->length != remain_size + 2))) {
			rsp_code = 0x13;
			break;
		}

		if ((uds_download.received_size >= uds_download.total_size) || \
		    (0 == uds_download.state)) {
			rsp_code = 0x24;
			break;
		}

		if (rxdata->data[1] != bsc) {
			rsp_code = 0x73;
			break;
		} else {
			bsc++;
		}
               /* specify the address in flash */
		dbuf = (uint8_t *)(uds_download.flash_base + uds_download.received_size);
		/*
		for (; idx < cur_xfer_size; idx++) {
			dbuf[idx] = rxdata->data[2 + idx];
		}
		*/
		memcpy(dbuf, &rxdata->data[2], cur_xfer_size);
		dcache_flush();

		if (0 == flash_memory_write((uint32_t)dbuf, &rxdata->data[2], cur_xfer_size)) {
			uds_download.received_size += cur_xfer_size;

			rsp_code = 0;
			txdata->data[0] = 0x76;
			txdata->data[1] = cur_bsc;
			txdata->length = 2;
		} else {
			rsp_code = 0x72;
			raw_writel(0xb30000, 0xde);
			raw_writel(0xb30000, 0xad);
			break;
		}
		/*After processing the last data, bsc set 1*/
               if(remain_size <= CAN_UDS_XFER_BLOCK_LEN - 2){
                      bsc =1;
		}
	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		txdata->data[0] = 0x7F;
		txdata->data[1] = 0x36;
		txdata->data[2] = rsp_code;
		txdata->length = 3;
	}
}

void can_uds_transfer_exit(pdu_t *rxdata, pdu_t *txdata, uint32_t pdu_id)
{
	uint8_t rsp_code = 0xFF;

	do {
		if ((NULL == rxdata) || (NULL == txdata)) {
			break;
		}

		uds_download.state = 0;

		rsp_code = 0;

		txdata->data[0] = 0x77;
		txdata->length = 1;

	} while (0);

	if ((0xFF != rsp_code) && (0 != rsp_code)) {
		/* service done. */
		txdata->data[0] = 0x7F;
		txdata->data[1] = 0x37;
		txdata->data[2] = rsp_code;
		txdata->length = 3;
	}
}
