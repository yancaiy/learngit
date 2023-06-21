/*
 * @Descripttion: 
 * @version: 
 * @Author: calterah
 * @Date: 2021-08-30 10:12:52
 * @LastEditors: haozhonglei
 * @LastEditTime: 2021-08-31 13:55:24
 */
#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "dw_i2c.h"

typedef struct
{
    uint32_t speed_mode;
    uint32_t address_mode;
    uint32_t slave_address;
    uint32_t reg_address;
    uint32_t reg_address_len;
    uint8_t *write_buf;
    uint8_t *read_buf;
    uint32_t data_len;
}I2C_Operation_Info;

typedef struct {

    uint32_t speed_mode; // 1 stand;2 fast;3 high;
    uint32_t slave_addr;
    uint32_t reg_addr_len;
    uint32_t reg_addr;
    uint32_t pin_mux; //0 default; 1 can0 IO; 2 can1 IO; 3 uart0 IO

} i2c_auto_test_config_t;

#define I2C_CID_1           1
#define I2C_CID_2           2
#define I2C_CID_3           3
#define I2C_CID_4           4
#define I2C_CID_5           5
#define I2C_CID_6           6

//test API
int32_t i2c_testcase_polling(I2C_Operation_Info *info);
// int32_t i2c_testcase2_polling_read(uint32_t address_mode, uint32_t slave_addr, uint32_t reg_addr, uint32_t reg_addr_len, uint8_t i2c_test_read_data[], uint32_t data_len);
// int32_t i2c_testcase3_int_transfer(uint32_t address_mode, uint32_t slave_addr, uint32_t reg_addr, uint32_t reg_addr_len, uint8_t i2c_test_transfer_data[], uint32_t data_len);
// int32_t i2c_testcase4_int_receive(uint32_t address_mode, uint32_t slave_addr, uint32_t reg_addr, uint32_t reg_addr_len, uint8_t i2c_test_receive_data[], uint32_t data_len);
// int32_t i2c_testcase5_polling_loop(uint32_t address_mode, uint32_t slave_addr, uint32_t reg_addr, uint32_t reg_addr_len, uint8_t i2c_test_write_data[], uint8_t i2c_test_read_data[], uint32_t data_len);

#endif