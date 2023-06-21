/***********************************************************************************************/
/**********************************BB DMA handshake test case***************************************/
/***********************************************************************************************/
#include "baseband.h"
#include "baseband_hw.h"
#include "baseband_alps_Mini_reg.h"
#include "debug.h"
#include "debug_reg.h"
#include "debug_hal.h"
#include "dma_hal.h"

#define BB_WRITE_REG(bb_hw, RN, val) baseband_write_reg(bb_hw, BB_REG_##RN, val)
#define BB_READ_REG(bb_hw, RN) baseband_read_reg(bb_hw, BB_REG_##RN)

/* make CHIP_VER=MINI BOARD=validation APPL=validation PLAT_ENV=FPGA BB_SHARED_MEM_EN=1 HILTEST=265 */
#define BB_LEN_WORD			(8192)

#define BB_REG_ADDR			(0x90000000)
#define CFG_DBG_DMA_STF		(0x3f4)
#define CFG_DBG_DMA_CFR		(0x3f8)
#define CFG_DBG_DMA_DOA		(0x3fc)
#define DMA_STF_ADDR		(BB_REG_ADDR + (CFG_DBG_DMA_STF << 2))
#define DMA_CFR_ADDR		(BB_REG_ADDR + (CFG_DBG_DMA_CFR << 2))
#define DMA_DOA_ADDR		(BB_REG_ADDR + (CFG_DBG_DMA_DOA << 2))

static uint32_t bb_dma_handshake_buf[BB_LEN_WORD];
static volatile uint32_t stft_dma_done = 0;
static volatile uint32_t cfar_dma_done = 0;
static volatile uint32_t doa_dma_done = 0;

extern bool baseband_hw_is_running(baseband_hw_t *bb_hw);
extern void baseband_hil_input_ahb(baseband_hw_t *bb_hw);

int32_t bb_dma_read(uint32_t *buf, uint32_t addr, uint32_t len,  void (*func)(void *), uint8_t handshake)
{
        int32_t result = E_OK;

        dma_trans_desc_t desc;

        do {
                if ((0 == len) || (NULL == buf)) {
                        result = E_PAR;
                        break;
                }

                if (len > DMAC_BLOCK_TS_MAX) {
                        desc.work_mode = DMA_SRC_NR_NG_LLP_DST_NR_NS_LLP;
                } else {
                        desc.work_mode = DMA_SRC_NR_NG_NLLP_DST_NR_NS_NLLP;
                }

                desc.transfer_type = PERI_TO_MEM;
                desc.priority = DMA_CHN_PRIOR_3;
                desc.block_transfer_size = len;

                desc.src_desc.burst_tran_len = BURST_LEN8;
                desc.src_desc.addr_mode = ADDRESS_FIXED;
                desc.src_desc.hw_if = handshake;
                desc.src_desc.sts_upd_en = 0;
                desc.src_desc.hs = HS_SELECT_HW;
                desc.src_desc.addr = addr;
                desc.src_tr_width = eTR_WIDTH_32_BITS;

                desc.dst_desc.burst_tran_len = BURST_LEN32;
                desc.dst_desc.addr_mode = ADDRESS_INCREMENT;
                desc.dst_desc.sts_upd_en = 0;
                desc.dst_desc.hs = HS_SELECT_SW;
                desc.dst_desc.addr = (uint32_t)buf;
                desc.dst_tr_width = eTR_WIDTH_32_BITS;

                result = dma_transfer(&desc, func);

        } while (0);

        return result;
}


static void stft_dma_done_callback(void *params)
{
        stft_dma_done = 1;

        EMBARC_PRINTF("stft_dma_done_callback Done\r\n");
        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void cfar_dma_done_callback(void *params)
{
        cfar_dma_done = 1;

        EMBARC_PRINTF("cfar_dma_done_callback Done\r\n");
        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void doa_dma_done_callback(void *params)
{
        doa_dma_done = 1;

        EMBARC_PRINTF("doa_dma_done_callback Done\r\n");
        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void set_DMA_stft(uint32_t stft_size)
{
        EMBARC_PRINTF("set_DMA_stft:src addr:0x%x, size:%d, handshake:%d\r\n", (uint32_t)DMA_STF_ADDR, stft_size, (uint8_t)DMA_REQ_BB_STF_RX);
        uint32_t loop = stft_size / DMAC_BLOCK_TS_MAX;
        uint32_t remainder = stft_size % DMAC_BLOCK_TS_MAX;

        for (uint32_t i = 0; i < loop; i++) {
                EMBARC_PRINTF("#########single: %d ##########\r\n", i);
                bb_dma_read(&bb_dma_handshake_buf[i * DMAC_BLOCK_TS_MAX], (uint32_t)DMA_STF_ADDR, DMAC_BLOCK_TS_MAX, stft_dma_done_callback, (uint8_t)DMA_REQ_BB_STF_RX);

                while (1){
                    if (stft_dma_done == 1) {
                            stft_dma_done = 0;
                            break;
                    } else {
                            //taskYIELD();
                            chip_hw_udelay(100);
                    }
                };
        }

        if (remainder != 0) {
                //remainder = 1016;
                EMBARC_PRINTF("######### remainder: %d ##########\r\n", remainder);
                bb_dma_read(&bb_dma_handshake_buf[loop * DMAC_BLOCK_TS_MAX], (uint32_t)DMA_STF_ADDR, remainder, stft_dma_done_callback, (uint8_t)DMA_REQ_BB_STF_RX);
                while(1) {
                         if (stft_dma_done == 1) {
                                 stft_dma_done = 0;
                                 break;
                         } else {
                                 //taskYIELD();
                                 chip_hw_udelay(100);
                         }
                };
         }

}


static void set_DMA_cfar(uint32_t cfar_size)
{
        EMBARC_PRINTF("set_DMA_cfar:src addr:0x%x, size:%d, handshake:%d\r\n", (uint32_t)DMA_CFR_ADDR, cfar_size, (uint8_t)DMA_REQ_BB_CFR_RX);
        uint32_t loop = cfar_size / DMAC_BLOCK_TS_MAX;
        uint32_t remainder = cfar_size % DMAC_BLOCK_TS_MAX;

        for (uint32_t i = 0; i < loop; i++) {
                EMBARC_PRINTF("#########single: %d ##########\r\n", i);
                bb_dma_read(&bb_dma_handshake_buf[i * DMAC_BLOCK_TS_MAX], (uint32_t)DMA_CFR_ADDR, DMAC_BLOCK_TS_MAX, cfar_dma_done_callback, (uint8_t)DMA_REQ_BB_CFR_RX);
                chip_hw_udelay(100);
                while (1){
                    if (cfar_dma_done == 1) {
                            cfar_dma_done = 0;
                            break;
                    } else {
                            //taskYIELD();
                            chip_hw_udelay(100);
                    }
                };
        }

        if (remainder != 0) {
                //remainder = 1016;
                EMBARC_PRINTF("######### remainder: %d ##########\r\n", remainder);
                bb_dma_read(&bb_dma_handshake_buf[loop * DMAC_BLOCK_TS_MAX], (uint32_t)DMA_CFR_ADDR, remainder, cfar_dma_done_callback, (uint8_t)DMA_REQ_BB_CFR_RX);
                while(1) {
                         if (cfar_dma_done == 1) {
                                 cfar_dma_done = 0;
                                 break;
                         } else {
                                 //taskYIELD();
                                 chip_hw_udelay(100);
                         }
                };
         }

}


static void set_DMA_doa(uint32_t doa_size)
{
        EMBARC_PRINTF("set_DMA_doa:src addr:0x%x, size:%d, handshake:%d\r\n", (uint32_t)DMA_DOA_ADDR, doa_size, (uint8_t)DMA_REQ_BB_DOA_RX);
        uint32_t loop = doa_size / DMAC_BLOCK_TS_MAX;
        uint32_t remainder = doa_size % DMAC_BLOCK_TS_MAX;

        for (uint32_t i = 0; i < loop; i++) {
                EMBARC_PRINTF("#########single: %d ##########\r\n", i);
                bb_dma_read(&bb_dma_handshake_buf[i * DMAC_BLOCK_TS_MAX], (uint32_t)DMA_DOA_ADDR, DMAC_BLOCK_TS_MAX, doa_dma_done_callback, (uint8_t)DMA_REQ_BB_DOA_RX);

                while (1){
                    if (doa_dma_done == 1) {
                            doa_dma_done = 0;
                            break;
                    } else {
                            //taskYIELD();
                            chip_hw_udelay(100);
                    }
                };
        }

        if (remainder != 0) {
                //remainder = 1016;
                EMBARC_PRINTF("######### remainder: %d ##########\r\n", remainder);
                bb_dma_read(&bb_dma_handshake_buf[loop * DMAC_BLOCK_TS_MAX], (uint32_t)DMA_DOA_ADDR, remainder, doa_dma_done_callback, (uint8_t)DMA_REQ_BB_DOA_RX);
                while(1) {
                         if (doa_dma_done == 1) {
                                 doa_dma_done = 0;
                                 break;
                         } else {
                                 //taskYIELD();
                                 chip_hw_udelay(100);
                         }
                };
         }

}


#if 0
static void set_DMA_cfar(uint32_t cfar_size)
{
        bb_dma_read(bb_dma_handshake_buf, (uint32_t)DMA_CFR_ADDR, cfar_size, cfar_dma_done_callback, (uint8_t)DMA_REQ_BB_CFR_RX);
}


static void set_DMA_doa(uint32_t doa_size)
{
        bb_dma_read(bb_dma_handshake_buf, (uint32_t)DMA_DOA_ADDR, doa_size, doa_dma_done_callback, (uint8_t)DMA_REQ_BB_DOA_RX);
}
#endif


void bb_test_doa_dma(void)
{
        baseband_t *bb = baseband_get_bb(1);

        dbgbus_rx_config_t hil_config = {
                .hil_src = eHIL_APB_BUS
        };

        debug_dbgbus_datahil(&hil_config);

        uint16_t bb_status_en =  SYS_ENA(HIL   , true)
                                |SYS_ENA(FFT_2D, true)
                                |SYS_ENA(CFR   , true);

        uint8_t bnk_act = BB_READ_REG(&bb->bb_hw, FDB_SYS_BNK_ACT); /* read back the bank selected in RTl */
        EMBARC_PRINTF("baseband_hw_start_with_params bnk_act = %d\r\n", bnk_act);
        BB_WRITE_REG(&bb->bb_hw, SYS_BNK_ACT, bnk_act);     /* match the bank accessed by CPU */
        /* start baseband */
        BB_WRITE_REG(&bb->bb_hw, SYS_ENABLE, bb_status_en);
        BB_WRITE_REG(&bb->bb_hw, SYS_IRQ_ENA, 0);

        //uint32_t stft_dma_size = BB_READ_REG(&bb->bb_hw, DBG_FDB_SIZE_DBG_TO_DMA_STFT);
        //EMBARC_PRINTF("stft_dma_size:0x%x \r\n", stft_dma_size);
        //set_DMA_stft(stft_dma_size);
        BB_WRITE_REG(&bb->bb_hw, SYS_START, 0x1);
        /* HIL data starts, sine wave via AHB bus*/
        void baseband_hil_input_ahb(baseband_hw_t *bb_hw);
        baseband_hil_input_ahb(&bb->bb_hw);

        bb_status_en = SYS_ENA(BFM   , true);
        BB_WRITE_REG(&bb->bb_hw, SYS_ENABLE, bb_status_en);
        /* Enable STFT/CFAR/DoA DMA data source */
        BB_WRITE_REG(&bb->bb_hw, DBG_CFG_DBG_TO_DMA_EN, 0x4);

        BB_WRITE_REG(&bb->bb_hw, SYS_START, 0x1);


        EMBARC_PRINTF("waiting to config DoA DMA \r\n");
        uint32_t doa_dma_size = BB_READ_REG(&bb->bb_hw, DBG_FDB_SIZE_DBG_TO_DMA_DOA);
        //doa_dma_size = 4200;
        EMBARC_PRINTF("doa_dma_size:0x%x \r\n", doa_dma_size); /* 19 objects * 360 per object */
        set_DMA_doa(doa_dma_size);

        /* wait done */
        int cnt = 0;
        while (baseband_hw_is_running(&bb->bb_hw) == true) {
                cnt++;
                if (cnt >= 65536*32) {
                        EMBARC_PRINTF("break due to cnter reaches its limint, so that debuger can check BB status by cil_task \r\n");
                        break;
                }
        }
        EMBARC_PRINTF("after DoA \r\n");

        /* parse cfar and doa reselt */
        baseband_parse_mem_rlt(&bb->bb_hw, true);
}


void bb_test_cfar_dma(void)
{
        for(uint32_t i = 0; i < BB_LEN_WORD; i++) {
            bb_dma_handshake_buf[i] = 0x2222a5a5;
        }

        baseband_t *bb = baseband_get_bb(1);

        /* Enable CFAR DMA data source */
        BB_WRITE_REG(&bb->bb_hw, DBG_CFG_DBG_TO_DMA_EN, 0x2);

        dbgbus_rx_config_t hil_config = {
                .hil_src = eHIL_APB_BUS
        };

        debug_dbgbus_datahil(&hil_config);

        uint16_t bb_status_en =  SYS_ENA(HIL   , true)
                                |SYS_ENA(FFT_2D, true)
                                |SYS_ENA(CFR   , true)
                                |SYS_ENA(BFM   , true);

        uint8_t bnk_act = BB_READ_REG(&bb->bb_hw, FDB_SYS_BNK_ACT); /* read back the bank selected in RTl */
        EMBARC_PRINTF("baseband_hw_start_with_params bnk_act = %d\r\n", bnk_act);
        BB_WRITE_REG(&bb->bb_hw, SYS_BNK_ACT, bnk_act);     /* match the bank accessed by CPU */
        /* start baseband */
        BB_WRITE_REG(&bb->bb_hw, SYS_ENABLE, bb_status_en);
        BB_WRITE_REG(&bb->bb_hw, SYS_IRQ_ENA, 0);

        BB_WRITE_REG(&bb->bb_hw, SYS_START, 0x1);

        /* HIL data starts, sine wave via AHB bus*/
        void baseband_hil_input_ahb(baseband_hw_t *bb_hw);
        baseband_hil_input_ahb(&bb->bb_hw);

        uint32_t cfar_dma_size = BB_READ_REG(&bb->bb_hw, DBG_FDB_SIZE_DBG_TO_DMA_CFR);
        EMBARC_PRINTF("cfar_dma_size:%d \r\n", cfar_dma_size);
        set_DMA_cfar(cfar_dma_size);

        EMBARC_PRINTF("bfe runing\r\n");

        /* wait done */
        int cnt = 0;
        while (baseband_hw_is_running(&bb->bb_hw) == true) {
                cnt++;
                if (cnt >= 65536*8) {
                        EMBARC_PRINTF("break due to cnter reaches its limint, so that debuger can check BB status by cil_task \r\n");
                        break;
                }
        }

        EMBARC_PRINTF("after runing\r\n");

        /* parse cfar and doa reselt */
        baseband_parse_mem_rlt(&bb->bb_hw, true);
}


void bb_test_stft_dma(void)
{
        baseband_t *bb = baseband_get_bb(1);
        BB_WRITE_REG(&bb->bb_hw, DBG_CFG_DBG_TO_DMA_EN, 0x1);
        dbgbus_rx_config_t hil_config = {
                .hil_src = eHIL_APB_BUS
        };

        debug_dbgbus_datahil(&hil_config);
        uint16_t bb_status_en =  SYS_ENA(HIL   , true)
                                |SYS_ENA(STFT  , true);
        uint8_t bnk_act = BB_READ_REG(&bb->bb_hw, FDB_SYS_BNK_ACT); /* read back the bank selected in RTl */
        EMBARC_PRINTF("baseband_hw_start_with_params bnk_act = %d\r\n", bnk_act);
        BB_WRITE_REG(&bb->bb_hw, SYS_BNK_ACT, bnk_act);     /* match the bank accessed by CPU */
        BB_WRITE_REG(&bb->bb_hw, SYS_ENABLE, bb_status_en);
        BB_WRITE_REG(&bb->bb_hw, SYS_IRQ_ENA, 0);
        BB_WRITE_REG(&bb->bb_hw, SYS_START, 0x1);
        uint32_t sys_status = BB_READ_REG(&bb->bb_hw, SYS_STATUS);
        EMBARC_PRINTF("bb status = %d\r\n", sys_status );
        void baseband_hil_input_ahb(baseband_hw_t *bb_hw);
        baseband_hil_input_ahb(&bb->bb_hw);

        uint32_t stft_dma_size = BB_READ_REG(&bb->bb_hw, DBG_FDB_SIZE_DBG_TO_DMA_STFT);
        EMBARC_PRINTF("stft_dma_size:%d \r\n", stft_dma_size);
        set_DMA_stft(stft_dma_size);
        EMBARC_PRINTF("end of test_STFT_DMA \r\n");

        sys_status = BB_READ_REG(&bb->bb_hw, SYS_STATUS);
        EMBARC_PRINTF("bb status = %d\r\n", sys_status );
         while (sys_status != 0) {
                EMBARC_PRINTF("bb status = %d\r\n", sys_status );
                sys_status = BB_READ_REG(&bb->bb_hw, SYS_STATUS);
                for (int i = 0; i<32; i++)
                    ;
        }
}
