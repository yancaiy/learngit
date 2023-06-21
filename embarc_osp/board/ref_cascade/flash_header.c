#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"

//#include "dw_qspi_reg.h"
#include "dw_ssi_reg.h"
#include "dw_ssi.h"
#include "xip_hal.h"

#include "flash_header.h"
#include "flash_mmap.h"

#include "instruction.h"
#include "vendor.h"

#include "config.h"


#if (defined SYSTEM_3_STAGES_BOOT) && (defined __FIRMWARE__)
__attribute__ ((aligned(4), section(".nvm_header")))
image_header_t sensor_flash_header = {
	.magic_number = 0x43616c74UL,
	.sw_version = {
		.major_ver_id = MAJOR_VERSION_ID,
		.minor_ver_id = MINOR_VERSION_ID,
		.stage_ver_id = STAGE_VERSION_ID,
		.date = __DATE__,
		.time = __TIME__,
		.info = SYS_NAME_STR
	},
	.payload_addr = FLASH_FIRMWARE_BASE + sizeof(image_header_t),
	//.payload_size
#if	FLASH_XIP_EN
	.xip_en = FLASH_XIP_ON,
#else
	.xip_en = FLASH_XIP_OFF,
#endif
	//.ram_base
	//.exec_offset
	.crc_part_size = 0x1000,
};
#else
__attribute__ ((aligned(4), section(".nvm_header")))
flash_header_t sensor_flash_header = {
	.magic_number = 0x43616c74UL,
	.version = 0,

    /* To ensure config external flash to quad mode successfully in ROM code */
    .qspi_speed0 = 1000000,

#if (defined SYSTEM_3_STAGES_BOOT)
	.pload_addr = FLASH_BOOT_BASE,
#else
	.pload_addr = FLASH_FIRMWARE_BASE,
#endif

	/* filled by script! */
	/*
	.pload_size = 0,
	.ram_base = 0,
	.exec_offset = 0,
	*/
#if	FLASH_XIP_EN
	.xip_flag = FLASH_XIP_ON,
	.pll_on = PLL_CLOCK_ON,
#else
	.xip_flag = FLASH_XIP_OFF,
	.pll_on = PLL_CLOCK_ON,
#endif

	/* TODO: while xip, the QSPI baud rate is fixed to 40Mbps! */
	.xip_ctrl = {
		//.ctrl0 = 0x5f0200,
		.ctrl0 = XIP_CONFIG_CTRL0,
		.ctrl1 = XIP_CONFIG_CTRL1,
		.baud = XIP_CONFIG_BAUD,
		.rx_sample_delay = XIP_CONFIG_RX_SAMPLE_DELAY,

		//.spi_ctrl = 0x3219,
		.spi_ctrl = XIP_CONFIG_SPI_CTRL,

		//.read_cmd = (0x1f << 16) | (0x6 << 21) | (0x2 << 25) | 0xeb,
		.read_cmd = XIP_CONFIG_READ_CMD,
		.xip_offset = XIP_CONFIG_INS_OFFSET,
		.ahb_endian = XIP_CONFIG_AHB_ENDIAN,
		.aes_endian = XIP_CONFIG_AES_ENDIAN,
		.xip_rx_buf_len = XIP_CONFIG_INS_RX_BUF_LEN,
		.data_offset = XIP_CONFIG_DATA_OFFSET,
		.data_rx_buf_len = XIP_CONFIG_DATA_RX_BUF_LEN,
#if FLASH_XIP_EN
		.mode = XIP_CONFIG_AES_MODE_XIP_ACCESS,
#else
		.mode = XIP_CONFIG_AES_MODE_CPU_ACCESS,
#endif
		.block_cnt = XIP_CONFIG_BLOCK_CNT,
		.last_block_size = XIP_CONFIG_LAST_BLOCK_SIZE
	},

	.cmd_sequence = {
		FLASH_DEV_CMD0, FLASH_DEV_CMD1, FLASH_DEV_CMD2, FLASH_DEV_CMD3
	},

	/* TODO: move to a separate file! */
	.pll_param = {
		/* (addr, value, delay). ms. */
		DEF_PLL_PARAM(0x00, 0x00, 0x00),
		DEF_PLL_PARAM(0x03, 0x07, 0x00),
		DEF_PLL_PARAM(0x05, 0xc0, 0x00),
		DEF_PLL_PARAM(0x09, 0x20, 0x00),
		DEF_PLL_PARAM(0x12, 0xc0, 0x00),
		DEF_PLL_PARAM(0x13, 0xc0, 0x00),
		DEF_PLL_PARAM(0x14, 0xc0, 0x00),
		DEF_PLL_PARAM(0x15, 0xb0, 0x00),
		DEF_PLL_PARAM(0x16, 0xb0, 0x00),
		DEF_PLL_PARAM(0x1b, 0xeb, 0x00),
		
#if FMCW_SDM_FREQ == 400
		DEF_PLL_PARAM(0x1d, 0x19, 0x00),
		DEF_PLL_PARAM(0x1e, 0xd0, 0x00),
#elif FMCW_SDM_FREQ == 450
		DEF_PLL_PARAM(0x1d, 0x18, 0x00),
		DEF_PLL_PARAM(0x1e, 0xc0, 0x00),
#elif FMCW_SDM_FREQ == 360
		DEF_PLL_PARAM(0x1d, 0x18, 0x00),
		DEF_PLL_PARAM(0x1e, 0xe0, 0x00),
#endif

		DEF_PLL_PARAM(0x25, 0xa0, 0x00),
		DEF_PLL_PARAM(0x26, 0x6f, 0x00),
		DEF_PLL_PARAM(0x7d, 0x00, 0x02),
#if FMCW_SDM_FREQ == 400
		DEF_PLL_PARAM(0x7d, 0x01, 0x07)
#elif FMCW_SDM_FREQ == 450
        DEF_PLL_PARAM(0x7d, 0x01, 0x07)
#elif FMCW_SDM_FREQ == 360
        DEF_PLL_PARAM(0x7d, 0x01, 0x0a)
#endif

	},

	.ahb_div = 0x1,
	.apb_div = 0x3,
	.qspi_speed = 4000000,

	.sec_dbg_cert_valid = SEC_DBG_CERT_INVLD,
	.sec_dbg_cert_addr = FLASH_DBG_CERT_BASE,
	.sec_dbg_cert_size = CONFIG_SEC_DBG_CERT_LEN,

	.pload_crc_granularity = 0x1000,

	.boot_timeout = 0xFFFFFFFFU,

	.sw_version = {
		.major_ver_id = MAJOR_VERSION_ID,
		.minor_ver_id = MINOR_VERSION_ID,
		.stage_ver_id = STAGE_VERSION_ID,
		.date = __DATE__,
		.time = __TIME__,
		.info = SYS_NAME_STR
	}
};
#endif

void *flash_header_get(void)
{
	void *pflash_header = &sensor_flash_header;

	return pflash_header;
}
