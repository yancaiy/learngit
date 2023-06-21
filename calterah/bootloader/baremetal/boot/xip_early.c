#include "xip_reg.h"
#include "dw_ssi_reg.h"


void flash_xip_init_early(void)
{
	xip_enable(1);

    /* QSPI related Register and XIP Register share the same start address - 0x00D00000U */
    /* The offset of QSPI related register is 0x0 */
    /* The offset of XIP related register is 0x100 */

    /* Configure QSPI related Register */
	raw_writel(REG_FLASH_XIP_BASE, XIP_CONFIG_CTRL0);
	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_CTRLR1_OFFSET), XIP_CONFIG_CTRL1);
	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_BAUDR_OFFSET), XIP_CONFIG_BAUD);
	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_RX_SAMPLE_DLY_OFFSET), XIP_CONFIG_RX_SAMPLE_DELAY);
	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_SPI_CTRLR0_OFFSET), XIP_CONFIG_SPI_CTRL);

	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_SER_OFFSET), 1);

	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_TXFTLR_OFFSET), 1);
	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_RXFTLR_OFFSET), 0);
	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_IMR_OFFSET), 0x3f);


    /* Configure XIP related Register */
	/* XIP Read Command Register - XRDCR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XRDCR_OFFSET), XIP_CONFIG_READ_CMD);

	/* XIP Instruction Section Offset Register - XISOR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XISOR_OFFSET), XIP_CONFIG_INS_OFFSET);

	/* XIP AHB Bus Endian Control Register - XABECR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XABECR_OFFSET), XIP_CONFIG_AHB_ENDIAN);

	/* XIP AES Data Bus Endian Control Register - XADBECR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XADBECR_OFFSET), XIP_CONFIG_AES_ENDIAN);

	/* AES Mode Register - AMR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_AMR_OFFSET), XIP_CONFIG_AES_MODE_XIP_ACCESS);

	/* XIP Instruction Buffer Control Register - XIBCR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XIBCR_OFFSET), CONFIG_XIP_INS_BUF_LEN);

	/* XIP Data Section Offset Register - XDSOR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XDSOR_OFFSET), CONFIG_XIP_DATA_OFFSET);

	/* XIP Read Buffer Control Register - XRBCR */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XRBCR_OFFSET), CONFIG_XIP_RD_BUF_LEN);

    /* Enable QSPI */
	raw_writel((REG_FLASH_XIP_BASE + REG_DW_SSI_SSIENR_OFFSET), 1);

	/* XIP Enable Register - XER */
	raw_writel((REG_FLASH_XIP_BASE + REG_FLASH_XIP_XER_OFFSET), 1);

	while (0 == raw_readl(REG_FLASH_XIP_BASE + REG_FLASH_XIP_XER_OFFSET)){
        chip_hw_mdelay(1);
    }
}
