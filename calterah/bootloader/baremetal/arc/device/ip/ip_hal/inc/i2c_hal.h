#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "dw_i2c.h"


typedef enum {
	I2C_INVALID_SPEED = 0,
	I2C_SS,
	I2C_FS,
	I2C_HS
} i2c_speed_t;

typedef enum {
	I2C_7BITS_ADDR = 0,
	I2C_10BITS_ADDR
} i2c_addr_mode_t;
#endif

#define I2C_M_RESTART_EN	(1)
#define I2C_M_RESTART_DIS	(0)

#define I2C_TX_EMPTY_CTRL_EN	(1)
#define I2C_TX_EMPTY_CTRL_DIS	(0)

#define I2C_TAR_SPECIAL_EN	(1)
#define I2C_TAR_SPECIAL_DIS	(0)

#define I2C_GENERAL_CALL	(0)
#define I2C_START_BYTE		(1)

#define I2C_TX_WATERMARK	(0)
#define I2C_RX_WATERMARK	(0)

#define I2C_MASTER_CODE		(0)

#define I2C_WRITE_CMD		(0)
#define I2C_READ_CMD		(1)

#define I2C_M_STOP_EN		(1)
#define I2C_M_STOP_DIS		(0)

typedef struct {
	uint32_t scl_l_cnt;
	uint32_t scl_h_cnt;
	uint16_t sda_rx_hold;
	uint16_t sda_tx_hold;
	uint32_t spike_len;
} i2c_io_timing_t;

typedef struct {
	uint32_t addr_mode;
	uint32_t speed_mode;
	uint32_t restart_en;
	uint32_t rx_timeout;//us

	i2c_io_timing_t *timing;
} i2c_params_t;

typedef enum {
	inaction = 0,
	transfer = 1,
	receive = 2,
}i2c_xfer_type_t;

typedef void(*xfer_callback)(void *);

int32_t i2c_init(uint32_t id, i2c_params_t *params);
int32_t i2c_transfer_config(uint32_t addr_mode, uint32_t slave_addr, uint32_t ext_addr, uint32_t ext_addr_len);
int32_t i2c_write(uint8_t *data, uint32_t len);
int32_t i2c_read(uint8_t *data, uint32_t len);
int32_t i2c_transfer_start(i2c_xfer_type_t xfer_type, uint8_t *data, uint32_t len, xfer_callback func);

int32_t i2c_fifo_threshold_config(i2c_xfer_type_t xfer_type, uint32_t thres);
int32_t i2c_interrupt_enable();
