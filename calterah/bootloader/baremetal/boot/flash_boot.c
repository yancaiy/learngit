#include "embARC.h"
#include "system.h"
#include "dw_ssi_reg.h"
#include "dw_ssi.h"
#include "flash.h"
#include "flash_header.h"
#include "flash_mmap.h"
#include "instruction.h"
#include "vendor.h"
#include "xip_hal.h"
#include "config.h"
#include "alps_clock.h"
#include "fmcw_radio_reset.h"
#include "calterah_error.h"

#ifndef BOOT_SPLIT
#include "xip_early.c"
#endif

void aes_init(void);
int aes_decrypt(uint32_t *data_in, uint32_t *data_out, uint32_t len);

#if (1 == BOOT_USE_XIP_LOAD_FW)
/*Use XIP to load FW from NOR flag*/
/*dst[OUT] : target buffer for data loading from NOR, 4 bytes aligned
  src[IN]  : source buffer in NOR for data loading, 4 bytes aligned
  len[IN]  : data length to be loaded, byte unit
  return : buffer with dst for success load or 0 for fail
*/
#define XIP_NOR_LOAD_ADDR_ALIGN       (4)
#define XIP_NOR_LOAD_ADDR_ALIGN_MASK  (XIP_NOR_LOAD_ADDR_ALIGN-1)
void *xip_nor_load(void* dst, void  *src, uint32_t len)
{
	unsigned int *pD,*pS;
	unsigned int nLen;

	/*align to 4bytes*/
	if (((unsigned int)dst & XIP_NOR_LOAD_ADDR_ALIGN_MASK) || 
		((unsigned int)src & XIP_NOR_LOAD_ADDR_ALIGN_MASK) ||
		(len%XIP_NOR_LOAD_ADDR_ALIGN)) {
		return 0;
    }
	/*length to be loaded with unit as word*/
	nLen = len/XIP_NOR_LOAD_ADDR_ALIGN;

	pD = (unsigned int *)dst;
	pS = (unsigned int *)src;

	while (nLen--) {
		*pD++ = *pS++;
	}

	return dst;
}
#endif /*1==BOOT_USE_XIP_LOAD_FW*/

int32_t normal_boot(int nBootFlag)
{
    int32_t result = E_CRC_MISMATCH;

    image_header_t image_header;
    image_header_t *image_header_ptr = (image_header_t *)&image_header;

    uint32_t crc32_size = 0;
    uint8_t *image_ptr = NULL;
    uint32_t *crc32_ptr = NULL;
    uint32_t crc32 = 0;
    uint32_t single_crc_size = 0;
    uint32_t payload_size = 0;
    uint32_t read_count;

    next_image_entry firmware_entry;

#ifdef BOOT_USE_HW_CRC
	/*HW CRC need setup time after enabled, move here for a delay inserted*/
	crc_enable(1);
#endif

    do {

#if (FLASH_XIP_EN)
        /* image header load with XIP */
        if (!xip_nor_load((void *)&image_header, (void*)(FLASH_MMAP_FIRMWARE_BASE + FLASH_FIRMWARE_BASE),sizeof(image_header_t))) {
            break;
        }
#else
        /* image header load with QSPI SSPI mode */
        result = flash_memory_read(FLASH_FIRMWARE_BASE, (uint8_t *)image_header_ptr, sizeof(image_header_t));
        if (E_OK != result) {
            break;
        }
        /* Check whether AES is enabled */
        if (raw_readl(REG_EMU_SEC_CTRL) & 1) {
            /* aes init process */
            aes_init();
            /* Start AES decrypt process to decrypt firmware image header */
            aes_decrypt((uint32_t *)image_header_ptr, (uint32_t *)image_header_ptr, sizeof(image_header_t));
        }
#endif

        /*Image Header SW CRC check*/
        crc32 = update_crc(0, (unsigned char *)&image_header, (sizeof(image_header) - 4));
        if (crc32 != image_header.crc32) {
            result = E_CRC_MISMATCH;
            break;
        }
        /* Boot firmware image under XIP mode */
        if (image_header_ptr->xip_en) {
#if (FLASH_XIP_EN)
            /* configure external nor flash to Quad mode and config XIP is done in ROM code */
#else
            /* send command to configure external nor flash to Quad mode. */
            result = flash_quad_entry();
            if (E_OK != result) {
                result = E_FLASH_QUAD_ENTRY;
                break;
            }
            /* configure XIP controller: */
            flash_xip_init_early();
#endif
            /* firmware memory mapping address */
            image_ptr = (uint8_t *)(FLASH_MMAP_FIRMWARE_BASE + image_header_ptr->payload_addr);
        } else {
            /* Boot firmware image under "copy to ram" mode */
            /* calculate the amount of crc */
            crc32_size = image_header_ptr->payload_size / image_header_ptr->crc_part_size;
            if (image_header_ptr->payload_size % image_header_ptr->crc_part_size) {
                crc32_size += 1;
            }
            crc32_size <<= 2;

            read_count = image_header_ptr->payload_size + crc32_size;
#if (1 != BOOT_USE_XIP_LOAD_FW)

            /* Check whether AES is enabled */
            if (raw_readl(REG_EMU_SEC_CTRL) & 1) {

                /* align read_count to 4 byte */
                if (image_header_ptr->payload_size % 0x4) {
                    read_count += 4 - (image_header_ptr->payload_size % 0x4);
                }

                /* align read_count to 64 byte. */
                if (read_count % 64) {
                    read_count += 64 - (read_count % 64);
                }

                /* Read firmware image from external flash and load the image data to RAM space at "ram_base" address */
                result = flash_memory_read(image_header_ptr->payload_addr, (uint8_t *)image_header_ptr->ram_base, read_count);
                if (E_OK != result) {
                    break;
                }

                /* aes init process */
                aes_init();

                /* Start AES decrypt process to decrypt firmware image */
                aes_decrypt((uint32_t *)image_header_ptr->ram_base, (uint32_t *)image_header_ptr->ram_base, read_count);
            } else {
                /* AES is disabled */
                /* Only need to Read firmware image from external flash and load image data to RAM space at "ram_base" address */
                result = flash_memory_read(image_header_ptr->payload_addr, (uint8_t *)image_header_ptr->ram_base, read_count);
                if (E_OK != result) {
                    break;
                }
            }
#else
/*XIP controller load FW from NOR flash*/
#define CAL_DATA_ALIGN      (0X40)             //64bytes align for AES decrypt
#define CAL_DATA_ALIGN_MASK (CAL_DATA_ALIGN-1) //64bytes align mask

#if (FLASH_XIP_EN)
            /* configure external nor flash to Quad mode and XIP config are done */
#else
            result = flash_quad_entry();
            if (E_OK != result) {
                /* Flash fail to enter quad mode */
                result = E_FLASH_QUAD_ENTRY;
                break;
            }
            /* If flash in quad mode, read image header using xip */
            /* configure XIP controller: */
            flash_xip_init_early();
#endif
            /*align to 64bytes*/
            read_count = ((read_count + CAL_DATA_ALIGN_MASK) & (~CAL_DATA_ALIGN_MASK));

            /*load FW to RAM with XIP controller*/
            if ( !xip_nor_load((void*)image_header.ram_base, 
                (void*)(FLASH_MMAP_FIRMWARE_BASE + image_header.payload_addr),
                read_count)) {
                result = E_CRC_MISMATCH;
                break;
            }
#endif
            /* firmware memory address */
            image_ptr = (uint8_t *)image_header.ram_base;
        }

        firmware_entry = (next_image_entry)(image_ptr + image_header_ptr->exec_offset);
        crc32_ptr = (uint32_t *)(image_ptr + image_header_ptr->payload_size);
        payload_size = image_header_ptr->payload_size;
        single_crc_size = image_header_ptr->crc_part_size;
        while (payload_size) {
            if (payload_size < single_crc_size) {
                single_crc_size = payload_size;
            }
#ifdef BOOT_USE_HW_CRC
            crc32 = hw_crc32_update(0, (unsigned int*)image_ptr, single_crc_size >> 2);
#else
            crc32 = update_crc(0, (unsigned char*)image_ptr, single_crc_size);
#endif
            if (crc32 != *crc32_ptr) {
                result = E_CRC_MISMATCH;
                break;
            }
            crc32_ptr++;
            image_ptr += single_crc_size;
            payload_size -= single_crc_size;
        }

        if (0 == payload_size) {
            _arc_aux_write(0x4B, 1);
            while (_arc_aux_read(0x48) & 0x100);
            icache_invalidate();

            /*for OTA verify check*/
            if (!nBootFlag) {
                result = E_OK;
            } else {
            /* jump to the next image. */
                firmware_entry();
            }
        }
    } while (0);

    return result;
}

void aes_init(void)
{
    /* Enable XIP clock */
    /* AES function is part of XIP module, so before use AES function, need to enable XIP clock */
    xip_enable(1);
    chip_hw_udelay(100);
    /* Set AES Mode Register - AMR */
    raw_writel(0xd00124, 0x2);
    /* Set AES Cipher/Decipher Mode Register - decipher mode */
    raw_writel(0xd00128, 1);
    /* Set AES Valid Block Register - AVBR */
    raw_writel(0xd00140, 4);
    /* Set AES Last Block Size Register - ALBSR */
    raw_writel(0xd00144, 0x80);
}

#define REG_FLASH_AES_DIN_OFFSET(x, y)	(((0x12 + ((x) << 2) + (y)) << 2))
#define REG_FLASH_AES_DOUT_OFFSET(x, y) (((0x26 + ((x) << 2) + (y)) << 2))
int aes_decrypt(uint32_t *data_in, uint32_t *data_out, uint32_t len)
{
    int ret = 0;

    unsigned int i, j;

    if (len % 64) {
        ret = -1;
    } else {
        while (len) {
            for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                raw_writel(REG_FLASH_AES_DIN_OFFSET(i, j) + 0xd00100, *data_in++);
            }
            }

            raw_writel(0xd00188, 1);
            raw_writel(0xd0018c, 1);
            //raw_writel(REG_FLASH_AES_DIN_VAL_OFFSET +XIP_ADDR, 1);

            while (0 == (raw_readl(0xd00194) & 0x1));
            for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                *data_out++ = raw_readl(REG_FLASH_AES_DOUT_OFFSET(i, j) + 0xd00100);
            }
            }
            len -= 64;
        }
    }

    return ret;
}

