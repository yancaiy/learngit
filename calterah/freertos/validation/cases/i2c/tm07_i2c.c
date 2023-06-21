#include "embARC_debug.h"
#include "dev_common.h"

#include "tm07_i2c.h"
#include "validation.h"
#include "vmodule.h"
#include "dw_i2c_reg.h"
#include "dw_i2c.h"
#include "i2c_hal.h"
#include "clkgen.h"
#include "alps_clock.h"
#include "gpio_hal.h"
#include "string.h"
#include "alps_dmu.h"

#define I2C_LOG		EMBARC_PRINTF
#define I2C_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_I2C_ID, cid, f_init, NULL)

#define I2C_TEST_DATA_LEN	512
static uint8_t test_data_buff_w[I2C_TEST_DATA_LEN] = {0};
static uint8_t test_data_buff_r[I2C_TEST_DATA_LEN] = {0};

i2c_io_timing_t i2c_timing_s =
{
    .scl_l_cnt   = 466,
    .scl_h_cnt   = 455,
    .sda_rx_hold = 0,
    .sda_tx_hold = 1,
    .spike_len   = 5,
};
i2c_io_timing_t i2c_timing_f =
{
    .scl_l_cnt   = 116,
    .scl_h_cnt   = 105,
    .sda_rx_hold = 0,
    .sda_tx_hold = 1,
    .spike_len   = 5,
};
i2c_io_timing_t i2c_timing_h =
{
    .scl_l_cnt   = 10,
    .scl_h_cnt   = 9,
    .sda_rx_hold = 0,
    .sda_tx_hold = 1,
    .spike_len   = 1,
};

int32_t i2c_testcase_polling(I2C_Operation_Info *info)
{
    int32_t result = E_OK;
    uint32_t id = 0;
    i2c_params_t i2c_test_params;
    do {
        i2c_test_params.addr_mode = info->address_mode;
        i2c_test_params.speed_mode = info->speed_mode;
        i2c_test_params.rx_timeout = 10000000;//10s
        i2c_test_params.restart_en = 1;
        if(info->speed_mode == I2C_SPPED_STANDARD_MODE)
        {
            i2c_test_params.timing = &i2c_timing_s;
        }
        else if(info->speed_mode == I2C_SPEED_FAST_MODE)
        {
            i2c_test_params.timing = &i2c_timing_f;
        }
        else
        {
            i2c_test_params.timing = &i2c_timing_h;
        }

        result = i2c_init(id, &i2c_test_params);
        if (E_OK != result) {
            break;
        }

        result = i2c_transfer_config(info->address_mode, info->slave_address, info->reg_address, info->reg_address_len);
        if (E_OK != result) {
            break;
        }

        result = i2c_write(info->write_buf, info->data_len);
        if (E_OK != result) {
        break;
        }

        result = i2c_transfer_config(info->address_mode, info->slave_address, info->reg_address, info->reg_address_len);
        if (E_OK != result) {
            break;
        }

        result = i2c_read(info->read_buf, info->data_len);
        if (E_OK != result) {
        break;
        }

        if(0 == memcmp(info->write_buf,info->read_buf,info->data_len))
        {
            result = E_OK;
        }
        else
        {
            result = -1;
        }

    } while (0);

    return result;
}

static int i2cIntTraState = 0; // 0 idle; 1 busy;
void i2c_interrupt_transfer_state_set()
{
    i2cIntTraState = 1;
}
void i2c_interrupt_transfer_state_clear(void *param)
{
    i2cIntTraState = 0;
}
void wait_interrupt_transfer_over()
{
    while(i2cIntTraState == 1)
    {
        chip_hw_mdelay(1);
    }
}

int32_t i2c_testcase_interrupt(I2C_Operation_Info *info)
{
    int32_t result = E_OK;
    uint32_t id = 0;
    i2c_params_t i2c_test_params;

    do {

        i2c_test_params.addr_mode = info->address_mode;
        i2c_test_params.speed_mode = info->speed_mode;
        i2c_test_params.rx_timeout = 10000000;//10s
        i2c_test_params.restart_en = 1;
        if(info->speed_mode == I2C_SPPED_STANDARD_MODE)
        {
            i2c_test_params.timing = &i2c_timing_s;
        }
        else if(info->speed_mode == I2C_SPEED_FAST_MODE)
        {
            i2c_test_params.timing = &i2c_timing_f;
        }
        else
        {
            i2c_test_params.timing = &i2c_timing_h;
        }

        result = i2c_init(id, &i2c_test_params);
        if (E_OK != result) {
            break;
        }

        result = i2c_fifo_threshold_config(1, 0);
        if (E_OK != result) {
            break;
        }

        result = i2c_transfer_config(info->address_mode, info->slave_address, info->reg_address, info->reg_address_len);
        if (E_OK != result) {
            break;
        }

        //interrupt--transmit
        result = i2c_interrupt_enable();
        if (E_OK != result) {
        break;
        }

        i2c_interrupt_transfer_state_set();

        result = i2c_transfer_start(1, info->write_buf, info->data_len, &i2c_interrupt_transfer_state_clear);
        if (E_OK != result) {
        break;
        }
        wait_interrupt_transfer_over();
        // chip_hw_mdelay(100);
        result = i2c_transfer_config(info->address_mode, info->slave_address, info->reg_address, info->reg_address_len);
        if (E_OK != result) {
            break;
        }

        //interrupt--transmit
        result = i2c_interrupt_enable();
        if (E_OK != result) {
        break;
        }

        i2c_interrupt_transfer_state_set();

        result = i2c_transfer_start(2, info->read_buf, info->data_len, &i2c_interrupt_transfer_state_clear);
        if (E_OK != result) {
        break;
        }

        wait_interrupt_transfer_over();
        // chip_hw_mdelay(100);
        if(0 == memcmp(info->write_buf,info->read_buf,info->data_len))
        {
            result = E_OK;
        }
        else
        {
            result = -1;
        }

    } while (0);

    return result;
}

/*
pin_mux : 0 defult; 1 adc_reset io; 2 adc_sync io; 3 pwm io;
*/
void i2c_io_mux(uint32_t pin_mux)
{
    switch (pin_mux)
    {
    case 0:
        io_mux_adc_reset_func_sel(0);
        io_mux_adc_sync_func_sel(0);
        io_mux_pwm0_func_sel(0);
        io_mux_pwm1_func_sel(0);
        io_mux_i2c_func_sel(0);
        break;
    case 1:
        io_mux_adc_sync_func_sel(0);
        io_mux_pwm0_func_sel(0);
        io_mux_pwm1_func_sel(0);
        io_mux_i2c_func_sel(4);
        io_mux_adc_reset_func_sel(2);
        break;
    case 2:
        io_mux_adc_reset_func_sel(0);
        io_mux_pwm0_func_sel(0);
        io_mux_pwm1_func_sel(0);
        io_mux_i2c_func_sel(4);
        io_mux_adc_sync_func_sel(1);
        break;
    case 3:
        io_mux_adc_reset_func_sel(0);
        io_mux_adc_sync_func_sel(0);
        io_mux_i2c_func_sel(4);
        io_mux_pwm0_func_sel(3);
        io_mux_pwm1_func_sel(3);
        break;
    default:
        io_mux_adc_reset_func_sel(0);
        io_mux_adc_sync_func_sel(0);
        io_mux_pwm0_func_sel(0);
        io_mux_pwm1_func_sel(0);
        io_mux_i2c_func_sel(0);
        break;
    }
}

/* TODO: add test cases */

int32_t i2c_case_handle(void *self, void *params, uint32_t len)
{
    validation_case_t *vscase_ptr = self;
    I2C_Operation_Info info;
    int32_t result = E_OK;
    i2c_auto_test_config_t config;
    param_analy(params,len,(auto_test_param_t *)&config);

    info.speed_mode = config.speed_mode;
    info.slave_address = config.slave_addr;
    info.reg_address_len = config.reg_addr_len;
    info.reg_address = config.reg_addr;
    info.write_buf = test_data_buff_w;
    info.read_buf = test_data_buff_r;
    info.data_len = I2C_TEST_DATA_LEN;

    for(int i = 0;i < I2C_TEST_DATA_LEN;i++)
    {
        test_data_buff_w[i] = i%256;
    }

    i2c_io_mux(config.pin_mux);

    switch (vscase_ptr->case_id)
    {
    case 1:
        info.address_mode = 0;
        result = i2c_testcase_polling(&info);
        break;
    case 2:
        info.address_mode = 0;
        result = i2c_testcase_interrupt(&info);
        break;
    case 3:
        info.address_mode = 1;
        result = i2c_testcase_polling(&info);
        break;
    case 4:
        info.address_mode = 1;
        result = i2c_testcase_interrupt(&info);
        break;
    case 6:
        info.address_mode = 0;
        result = i2c_testcase_interrupt(&info);
        break;
    default:
        return -1;
        break;
    }
    return result;
}

I2C_VALIDATION_SYS_CASE(I2C_CID_1, i2c_case_handle);
I2C_VALIDATION_SYS_CASE(I2C_CID_2, i2c_case_handle);
I2C_VALIDATION_SYS_CASE(I2C_CID_3, i2c_case_handle);
I2C_VALIDATION_SYS_CASE(I2C_CID_4, i2c_case_handle);
I2C_VALIDATION_SYS_CASE(I2C_CID_5, i2c_case_handle);
I2C_VALIDATION_SYS_CASE(I2C_CID_6, i2c_case_handle);
