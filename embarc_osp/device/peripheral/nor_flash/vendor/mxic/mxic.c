#include "embARC.h"
#include "embARC_debug.h"
//#include "dw_qspi.h"
#include "dw_ssi.h"
#include "device.h"
#include "xip_hal.h"

#include "instruction.h"
#include "sfdp.h"

/* us. */
#define SUSPEND_LATENCY_MAX		(40)
/* ms. */
#define RETRY_CNT				(50)


static int32_t mxic_param_read(dw_ssi_t *dw_ssi);
static int32_t mxic_quad_entry(dw_ssi_t *dw_ssi);
static int32_t mxic_quad_exit(dw_ssi_t *dw_ssi);
static int32_t mxic_qpi_entry(dw_ssi_t *dw_ssi);
static int32_t mxic_qpi_exit(dw_ssi_t *dw_ssi);
static int32_t mxic_status_read(dw_ssi_t *dw_ssi, uint32_t status);

static flash_dev_ops_t mxic_ops = {
	.quad_entry = mxic_quad_entry,
	.quad_exit = mxic_quad_exit,
	.qpi_entry = mxic_qpi_entry,
	.qpi_exit = mxic_qpi_exit,

	.status = mxic_status_read
};

#ifdef STATIC_EXT_FLASH_PARAM
static flash_device_t mxic_dev = {
	.total_size = 32000000 >> 3,
	.page_size = 256,
	.m_region = {
		DEF_FLASH_REGION(0x0, 0x400000, 0x400000, 0x20)
	},
	//.m_region_cnt = sizeof(flash_m_region)/sizeof(flash_region_t),
	.ops = &mxic_ops
};
#else
static flash_device_t mxic_dev;
#endif

/* description: called during boot. */
flash_device_t *get_flash_device(dw_ssi_t *dw_ssi)
{
	int32_t result = E_OK;

	static uint32_t ext_dev_open_flag = 0;

	flash_device_t *ext_dev = NULL;

#ifdef STATIC_EXT_FLASH_PARAM
	ext_dev = &mxic_dev;
#else
	if (0 == ext_dev_open_flag) {
		result = mxic_param_read(dw_ssi);
		if (E_OK == result) {
			ext_dev = &mxic_dev;
			ext_dev->ops = &mxic_ops;
		}
	} else {
		ext_dev = &mxic_dev;
	}
#endif

	return ext_dev;
}

static int32_t mxic_param_read(dw_ssi_t *dw_ssi)
{
	int32_t result = E_OK;

	if ((NULL == dw_ssi) || (NULL == dw_ssi->ops)) {
		result = E_PAR;
	} else {
		result = flash_sfdp_detect(dw_ssi);
		if (result > 0) {
			result = flash_sfdp_param_read(dw_ssi, \
					&mxic_dev);
		} else {
			if (E_OK == result) {
				result = E_NOEXS;
			}
		}
	}

	return result;
}

static int32_t mxic_quad_entry(dw_ssi_t *dw_ssi)
{
	int32_t result = E_OK;

	uint32_t cpu_status = 0;
	uint8_t wdata[1] = { 0x40};
	int8_t cnt = RETRY_CNT;

	/* Write Status Register 2(QE bit) - 0x31h */
	/* Write Enable Command            - 0x06h */
	dw_ssi_xfer_t xfer = DW_SSI_XFER_INIT(CMD_WRITE_STS1_CFG1, 0, 8, 0);
	dw_ssi_xfer_t we_xfer = DW_SSI_XFER_INIT(CMD_WRITE_ENABLE, 0, 0, 0);

	do {
		if ((NULL == dw_ssi) || (NULL == dw_ssi->ops)) {
			result = E_PAR;
			break;
		} else {
			dw_ssi_ops_t *ssi_ops = (dw_ssi_ops_t *)dw_ssi->ops;

			cpu_status = arc_lock_save();
			result = ssi_ops->write(dw_ssi, &we_xfer, 0);
			arc_unlock_restore(cpu_status);

			/* Wait at most 50ms until FLASH_WEL is set to 1 */
			while ((E_OK == result) && (cnt--)) {
				chip_hw_mdelay(1);
				result = mxic_status_read(dw_ssi, FLASH_WEL);
			}
			if (!result) {
				result = E_TMOUT;
				break;
			}

			xfer.buf = (void *)wdata;
			xfer.len = 1;
			result = ssi_ops->write(dw_ssi, &xfer, 0);
			cnt = RETRY_CNT;

			/* Wait at most 50ms until FLASH_WIP is set to 0 */
			do {
				chip_hw_mdelay(1);
				result = mxic_status_read(dw_ssi, FLASH_WIP);
			} while ((E_OK != result) && (cnt--));
		}
	} while (0);

	return result;
}

static int32_t mxic_quad_exit(dw_ssi_t *dw_ssi)
{
	int32_t result = E_OK;

	return result;
}

static int32_t mxic_qpi_entry(dw_ssi_t *dw_ssi)
{
	return E_NOEXS;
}

static int32_t mxic_qpi_exit(dw_ssi_t *dw_ssi)
{
	return E_NOEXS;
}

static int32_t mxic_status_read(dw_ssi_t *dw_ssi, uint32_t status)
{
	int32_t result = E_OK;

	uint8_t dev_status = 0;
	uint8_t sts_pos = 0;
	/* Frame Format mode */
	uint32_t frf_mode = 0;
	dw_ssi_ops_t *ssi_ops = NULL;
	dw_ssi_xfer_t xfer = DW_SSI_XFER_INIT(0, 0, 0, 0);

	if ((NULL == dw_ssi) || (NULL == dw_ssi->ops)) {
		result = E_PAR;
	} else {
		ssi_ops = (dw_ssi_ops_t *)dw_ssi->ops;

		xfer.buf = (void *)&dev_status;
		xfer.len = 1;

		switch (status) {
			case FLASH_WIP:
			case FLASH_WEL:
				xfer.ins = 0x05;
				sts_pos = 1 << status;
			case FLASH_BP:
			case FLASH_E_ERR:
			case FLASH_P_ERR:
			case FLASH_STSR_WD:
				xfer.ins = CMD_READ_RSTS1;
				sts_pos = 1 << status;
				break;
			case FLASH_P_SUSPEND:
			case FLASH_E_SUSPEND:
				xfer.ins = CMD_READ_RSTS2;
				sts_pos = 1 << (status - 8);
				break;
			default:
				result = E_PAR;
		}

		/* When it is in quad mode now, you need to switch to standard SPI mode
		to read the flash status, and then switch back to quad mode */
		frf_mode = ssi_ops->frf_get(dw_ssi);
		if (frf_mode == QUAD_FRAME_FORMAT) {
			ssi_ops->frf_update(dw_ssi, STANDARD_SPI_FRAME_FORMAT);
			result = ssi_ops->read(dw_ssi, &xfer, 0);
			if (E_OK == result) {
				if (sts_pos & dev_status) {
					result = 1;
				}
			}
			ssi_ops->frf_update(dw_ssi, QUAD_FRAME_FORMAT);
		}
		else {
			result = ssi_ops->read(dw_ssi, &xfer, 0);
			if (E_OK == result) {
				if (sts_pos & dev_status) {
					result = 1;
				}
			}
		}
	}


	return result;
}

#if 0
int32_t mxic_status_read_test(dw_ssi_t *dw_ssi)
{
	int32_t result = E_OK;

	uint8_t dev_status[4] = {0xa5, 0xa5, 0xa5, 0xa5};
	dw_ssi_xfer_t xfer = DW_SSI_XFER_INIT(CMD_READ_RSTS1, 0, 0, 0);

	dw_ssi_ops_t *ssi_ops = NULL;

	if ((NULL == dw_ssi) || (NULL == dw_ssi->ops)) {
		result = E_PAR;
	} else {
		ssi_ops = (dw_ssi_ops_t *)dw_ssi->ops;

		xfer.buf = (void *)&dev_status;
		xfer.len = 2;

		result = ssi_ops->readb(dw_ssi, &xfer);
		if (E_OK == result) {
			EMBARC_PRINTF("fls_dev_sts: 0x%x, 0x%x, 0x%x, 0x%x\r\n", \
					dev_status[0],\
					dev_status[1],\
					dev_status[2],\
					dev_status[3]\
					);
		}
	}

	return result;
}
#endif

static xip_param_t xip_info = {
	.rd_sts_cmd = FLASH_COMMAND(CMD_READ_RSTS1, \
			DW_SSI_INS_LEN_8, DW_SSI_ADDR_LEN_0, DW_SSI_DATA_LEN_8),
	.rd_cmd = FLASH_COMMAND(CMD_Q_READ, \
			DW_SSI_INS_LEN_8, DW_SSI_ADDR_LEN_24, DW_SSI_DATA_LEN_32),
	.wr_en_cmd = FLASH_COMMAND(CMD_WRITE_ENABLE, \
			DW_SSI_INS_LEN_8, DW_SSI_ADDR_LEN_0, DW_SSI_DATA_LEN_8),
	.program_cmd = FLASH_COMMAND(CMD_PAGE_PROG, \
			DW_SSI_INS_LEN_8, DW_SSI_ADDR_LEN_24, DW_SSI_DATA_LEN_32),
	.suspend_cmd = FLASH_COMMAND(CMD_PGSP, \
			DW_SSI_INS_LEN_8, DW_SSI_ADDR_LEN_0, DW_SSI_DATA_LEN_8),
	.resume_cmd = FLASH_COMMAND(CMD_PGRS, \
			DW_SSI_INS_LEN_8, DW_SSI_ADDR_LEN_0, DW_SSI_DATA_LEN_8),

	.wip_pos = (1 << 0),
	.xsb = (1 << 8) | (2 << 16),
	.suspend_wait = 0x1ff
};

xip_param_t *flash_xip_param_get(void)
{
	uint32_t freq = clock_frequency(XIP_CLOCK);

	xip_info.suspend_wait = 40 * (freq / 1000000);

	return &xip_info;
}
