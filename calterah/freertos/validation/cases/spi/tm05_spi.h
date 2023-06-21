#ifndef TM05_SPI_H
#define TM05_SPI_H

#define SPI_CID_1           1

//spim test case
int32_t spim_test_polling_case1(uint32_t id, uint32_t baud_rate, uint32_t *data, uint32_t len);
int32_t spim_test_int_transfer_case2(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len);
int32_t spim_test_int_receive_case2(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len);
int32_t spim_test_DMA_write_case4(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len);
int32_t spim_test_DMA_read_case4(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len);
//spis test case
int32_t spis_test_polling_read_case11(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len);
int32_t spis_test_polling_write_case11(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len);
int32_t spis_test_int_transfer_case2(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len);
int32_t spis_test_int_receive_case2(uint32_t id, uint32_t baud_rate, uint32_t *readdata, uint32_t len);

//qspi test case
int32_t spiq_test_polling_flash_case7(uint32_t addr, uint8_t *wdata, uint8_t *rdata, uint32_t len);
int32_t spiq_test_polling_flasherase_case7(uint32_t addr, uint32_t len);
int32_t spiq_test_int_transfer_case10(uint8_t *writedata, uint32_t len, uint32_t addr);
int32_t spiq_test_int_receive_case10(uint8_t *readdata, uint32_t len, uint32_t addr);
//qspi<->dma test case
int32_t spiq_test_dma_write_case11(uint32_t addr, uint32_t *wdata, uint32_t len);
int32_t spiq_test_dma_read_case11(uint32_t addr, uint32_t *rdata, uint32_t len);

#if 0
int32_t spim_test_polling_case5(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t *readdata, uint32_t len);
//spim<->flash test case
int32_t spim_test_polling_flashwrite_case6(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t len);
int32_t spim_test_flashchiperase(uint32_t id, uint32_t baud_rate);
int32_t spim_test_polling_flashread_case6(uint32_t id, uint32_t baud_rate, uint32_t *writedata, uint32_t *readdata, uint32_t len);
#endif
#endif
