/*
 * @Author: your name
 * @Date: 2020-06-17 20:08:30
 * @LastEditTime: 2022-04-08 13:54:45
 * @LastEditors: fang yongjun
 * @Description: In User Settings Edit
 * @FilePath: can_dbg.c
 */
#include <string.h>
#include "can_dbg.h"
#include "baseband.h"
#include "semphr.h"
#include "baseband_alps_FM_reg.h"
#include "dbg_gpio_reg.h"
#include "baseband_task.h"
#include "sensor_config.h"
#include "embARC.h"
#include "can_hal.h"
#include "can_trans.h"
#include "baseband_reg.h"
#include "dev_can.h"
#include "sharedDef.h"
//#include "includes_app_h.h"
#include "radardsp.h"
#include "sensor_config_cli.h"

CAN_DBG_T can_dbg_mode;

static u16 gRecvCmdStarted = 0;
static u16 gRecvCmdLen;
static u16 gRecvCmdIdx;
static u16 gRecCmdCnt = 0;

u8 cfg_mode = 0; //进入配置模式，不上报数据 0：正常模式， 1：配置模式

#define CAN_CMD_REC_BUF 600
static char gCmdBuf[CAN_CMD_REC_BUF];
static char gCmdAck[CAN_CMD_REC_BUF];

typedef enum
{
    eBB_DATDUMP_ADC = 0,   /* Dump ADC Data */
    eBB_DATDUMP_1DFFT = 1, /* Dump 1DFFT Data */
    eBB_DATDUMP_2DFFT = 2, /* Dump 2DFFT Data */
    eBB_DATDUMP_CFAR = 3,  /* Dump CFAR Data */
    eBB_DATDUMP_BFM = 4,   /* Dump BFM Data */
} eBB_DATDUMP_TYPE;

typedef enum
{
    eBB_DATDUMP_SPI = 0,  /* Dump BB Data through SPI Interface */
    eBB_DATDUMP_UART = 1, /* Dump BB Data through UART Interface */
    eBB_DATDUMP_CAN = 2,  /*Dump BB Data through CAN Interface */
} eBB_DATDUMP_IF;

uint8_t isCfgMode(void)
{
    return cfg_mode;
}

static void send_can_cmd_ack(uint8_t canIdx, char *ack)
{
    int id = CMD_FRAME_ID;
    int idx = 0;
    int len = strlen(ack);
    int cnt = (len + 6) / 7;
    char data[8] = {
        0,
    };

    data[0] = 0x88;
    data[1] = len >> 8;
    data[2] = len & 0xFF;
    gCAN[canIdx].sendFrameNow(data, id, 8);

    for (int i = 0; i < cnt; i++)
    {
        data[0] = i & 0xF;

        for (int j = 1; j < 8 && idx < len; j++)
        {
            data[j] = ack[idx++];
        }

        gCAN[canIdx].sendFrameNow(data, id, 8);
    }

    data[0] = 0xFF;

    gCAN[canIdx].sendFrameNow(data, id, 8);
}

int32_t recv_can_cmd(uint8_t canIdx, uint8_t *data, uint32_t len)
{
    if (data[0] == 0x88)
    {
        gRecCmdCnt++;
        EMBARC_PRINTF("cmd - %d\r\n", gRecCmdCnt);
        gRecvCmdIdx = 0;
        gRecvCmdLen = data[1] << 8 | data[2];

        if (gRecvCmdLen > CAN_CMD_REC_BUF)
        {
            return 0;
        }
        gRecvCmdStarted = 1;
    }
    else if (data[0] == 0xFF)
    {
        gCmdBuf[gRecvCmdLen] = 0;

        sensor_cfg_command_handler(gCmdAck, CAN_CMD_REC_BUF, gCmdBuf);
        send_can_cmd_ack(canIdx,gCmdAck);
        gCmdAck[0] = 0;
        gRecvCmdStarted = 0;
    }
    //ï¿½å»ºï¿½ï¿½
    else if (data[0] == 0xC1)
    {
        gRecvCmdStarted = 0;
        gRecvCmdIdx = 0;
        gRecvCmdLen = 0;
        gRecCmdCnt = 0;
        sensor_cfg_command_handler(gCmdAck, CAN_CMD_REC_BUF, "clear\n");
        send_can_cmd_ack(canIdx,gCmdAck);
        gCmdAck[0] = 0;
    }
    else if ((gRecvCmdIdx < gRecvCmdLen) && (gRecvCmdStarted))
    {
        memcpy(gCmdBuf + gRecvCmdIdx, data + 1, 7);
        gRecvCmdIdx += 7;
    }
    return 0;
}


void send_dbg_head(int ch_index, int rng, int vel)
{
    char data[8] = {
        0,
    };

    data[0] = 0x10;
    data[1] = ch_index;
    data[2] = rng >> 8;
    data[3] = rng & 0xFF;
    data[4] = vel >> 8;
    data[5] = vel & 0xFF;
    gCAN[PCAN].sendFrameNow(data, CAN_DBG_ID, 8);
}

void send_dbg_end(void)
{
    char data[8] = {
        0,
    };

    data[0] = 0x70;
    gCAN[PCAN].sendFrameNow(data, CAN_DBG_ID, 8);
}

void send_req_ack(uint8_t canIdx)
{
    char data[8] = {
        0,
    };

    data[0] = 0x81;
    gCAN[canIdx].sendFrameNow((uint32_t *)(data), CAN_DBG_ID, 8);
}

void send_req_mode_ack(uint8_t canIdx)
{
    char data[8] = {
        0,
    };

    data[0] = 0x61;
    gCAN[canIdx].sendFrameNow((uint32_t *)(data), CAN_DBG_ID, 8);
}

void send_dbg_data(uint32_t *src_data, uint32_t len, uint8_t start_flag)
{
    static uint8_t frame_idx = 0;
    int i = 0, j = 0;
    if (start_flag)
    {
        frame_idx = 0;
    }
    if ((len <= 0) || (src_data == NULL))
    {
        return;
    }
    char fdata[28] = {
        0,
    };

    for (i = 0, j = 0; i < len; i++)
    {
        fdata[j++] = (src_data[i] >> 24) & 0x000000FF;
        fdata[j++] = (src_data[i] >> 16) & 0x000000FF;
        fdata[j++] = (src_data[i] >> 8) & 0x000000FF;
        fdata[j++] = src_data[i] & 0x000000FF;
    }

    for (i = 0; i < j;)
    {
        char data[8] = {
            0,
        };
        data[0] = 0x20 | (frame_idx & 0x0f);
        frame_idx++;
        data[1] = fdata[i++];
        data[2] = fdata[i++];
        data[3] = fdata[i++];
        data[4] = fdata[i++];
        data[5] = fdata[i++];
        data[6] = fdata[i++];
        data[7] = fdata[i++];

        gCAN[PCAN].sendFrameNow((uint32_t *)(data), CAN_DBG_ID, 8);
    }
}

int32_t can_cmd_headler_dbg_cmd(uint8_t canIdx, uint8_t *data, uint32_t len)
{
    if (data[0] == 0x80)
    {
        can_dbg_mode.cur_frame_type = data[1];
        can_dbg_mode.ch = data[2];
        can_dbg_mode.dat_type = data[3];
        send_req_ack(canIdx);
//        newParam_flag_set(NEW_PARAM_ADC);
        EMBARC_PRINTF("output adc f=%d ,ch=%d ,ty=%d\n", can_dbg_mode.cur_frame_type, can_dbg_mode.ch, can_dbg_mode.dat_type);
        return 0;
    }
    else if (data[0] == 0x60)
    {
        cfg_mode = 1;
        EMBARC_PRINTF(" adc mode \n");
        send_req_mode_ack(canIdx);
    }

    return 0;
}

int can_bb_datdump_serport_command_handler(int cur_frame_type, int ch, int dat_type)
{
    uint8_t frame_type = baseband_get_cur_frame_type();
    baseband_t *bb = baseband_get_rtl_frame_type(); /* read back the bank selected in RTL */
    sensor_config_t *cfg = (sensor_config_t *)(bb->cfg);
    baseband_hw_t *bb_hw = &bb->bb_hw;
    int chirpNum = cfg->vel_nfft;

    unsigned char dump_position = 0;

    eBB_DATDUMP_TYPE eBBDatdumpType;               // 0: adc  1: fft1d  2: fft2d  3: cfar  4: bfm
    eBB_DATDUMP_IF eBBDatdumpIf = eBB_DATDUMP_CAN; // 0: SPI  1: UART 2 : CAN
    char bpm_index_input = 0;                      // Input bpm (virtual array) index of adc/fft1d/fft2d data to dump to UART
    signed char ch_index_input = 0;                // Input channel index of adc/fft1d/fft2d data to dump to UART
    char *dat_type_str[3] = {"\nADC", "\nFFT_1D", "\nFFT_2D"};

    /*确认是否是需要获取的frame*/
    if (frame_type != cur_frame_type)
    {
        return 0;
    }

    /* Get Data Dump Type Parameter - ADC/1DFFT/2DFFT/CFAR/BFM */

    if (dat_type == eBB_DATDUMP_ADC)
    {
        chirpNum = cfg->nchirp;
        dump_position = SYS_BUF_STORE_ADC;
        eBBDatdumpType = eBB_DATDUMP_ADC;
    }
    else if (dat_type == eBB_DATDUMP_1DFFT)
    {
        dump_position = SYS_BUF_STORE_FFT;
        eBBDatdumpType = eBB_DATDUMP_1DFFT;
    }
    else if (dat_type == eBB_DATDUMP_2DFFT)
    {
        dump_position = SYS_BUF_STORE_FFT;
        eBBDatdumpType = eBB_DATDUMP_2DFFT;
    }
    else if (dat_type == eBB_DATDUMP_CFAR)
    {
        dump_position = SYS_BUF_STORE_FFT; // This is a MUST! Or ERROR will happen!
        eBBDatdumpType = eBB_DATDUMP_CFAR;
    }
    else if (dat_type == eBB_DATDUMP_BFM)
    {
        dump_position = SYS_BUF_STORE_FFT; // This is a MUST! Or ERROR will happen!
        eBBDatdumpType = eBB_DATDUMP_BFM;
    }
    else
    {
        return 0;
    }

    /* specify BPM and channel index using uart dump ADC/1D-FFT/2D-FFT data */
    ch_index_input = ch;
    if (ch_index_input > 3 || ch_index_input < 0)
    {
        ch_index_input = -1; // If param4 > 3, print all 4 channel data!
    }
    /* bb run */
    bool tx_en = true;
    bool radio_en = true;
    /* baseband dump init */
    uint32_t old_buf_store = baseband_switch_buf_store(bb_hw, dump_position);
    uint8_t old_bnk = BB_READ_REG(bb_hw, FDB_SYS_BNK_ACT); /* read back the bank selected in RTl */
    uint16_t old_status_en = BB_READ_REG(bb_hw, SYS_ENABLE);
    uint16_t bb_status_en = 0;
    uint8_t bb_mem_access_pos;

    if (eBBDatdumpType == eBB_DATDUMP_2DFFT)
    {
        /* Set BB run until 2DFFT finish */
        bb_status_en = (1 << SYS_ENABLE_SAM_SHIFT) | (1 << SYS_ENABLE_FFT_2D_SHIFT);
        bb_mem_access_pos = SYS_MEM_ACT_BUF;
    }
    else if ((eBBDatdumpType == eBB_DATDUMP_ADC) || (eBBDatdumpType == eBB_DATDUMP_1DFFT))
    {
        /* Set BB run until ADC/1DFFT Sample finish */
        bb_status_en = (1 << SYS_ENABLE_SAM_SHIFT);
        bb_mem_access_pos = SYS_MEM_ACT_BUF;
    }
    else if ((eBBDatdumpType == eBB_DATDUMP_CFAR) || (eBBDatdumpType = eBB_DATDUMP_BFM))
    {
        /* Set BB run until CFAR/BFM finish - Only UART Support */
        bb_status_en = (SYS_ENABLE_SAM_MASK << SYS_ENABLE_SAM_SHIFT) | (SYS_ENABLE_FFT_2D_MASK << SYS_ENABLE_FFT_2D_SHIFT) | (SYS_ENABLE_CFR_MASK << SYS_ENABLE_CFR_SHIFT) | (SYS_ENABLE_BFM_MASK << SYS_ENABLE_BFM_SHIFT);
        bb_mem_access_pos = SYS_MEM_ACT_RLT;
    }

    /*--------------------------- dump data --------------------------*/
    EMBARC_PRINTF("\n # data_type is %d, dump_type is %d\n", eBBDatdumpType, eBBDatdumpIf);

    EMBARC_PRINTF("\n # current frame_type = %d \n", frame_type);

    /* dump adc/fft1d/fft2d data through uart */
    if (((eBBDatdumpType == eBB_DATDUMP_ADC) || (eBBDatdumpType == eBB_DATDUMP_1DFFT) || (eBBDatdumpType == eBB_DATDUMP_2DFFT)) && (eBBDatdumpIf == eBB_DATDUMP_CAN))
    {
        int ch_index, vel_index, rng_index;
        uint32_t fft_mem;
        uint32_t old;
        unsigned char dump_channel_num = 1;

        if (ch_index_input < 0)
        {
            dump_channel_num = MAX_NUM_RX; // print all 4 channels
        }
        else
        {
            dump_channel_num = 1; // print one channel
        }
        EMBARC_PRINTF("# ch_index_input = %d, dump_channel_num = %d \n", ch_index_input, dump_channel_num);

        {
            int data_idx = 0;
            /* 7 个数据整理发送一次，节省传输次数时间 */
            uint32_t data_buf[7];
            /* BB Start */
            baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
            baseband_start_with_params(bb, radio_en, tx_en, bb_status_en, true, BB_IRQ_ENABLE_ALL, false);
            /* wait queue for last BB HW run*/
            BASEBAND_ASSERT(baseband_wait_bb_done(INT_MODE, DEFAULT_TIMOUT) == E_OK);
            old = baseband_switch_mem_access(bb_hw, bb_mem_access_pos);

            for (ch_index = 0; ch_index < dump_channel_num; ch_index++)
            {
                EMBARC_PRINTF(dat_type_str[eBBDatdumpType]);
                EMBARC_PRINTF("(:, :, %d, %d) = [ ", ch_index + 1, bpm_index_input + 1); // print data in MATLAB style

                data_idx = 0;

                /* 发送头数据包 */
                send_dbg_data(NULL, 0, 1);
                if (ch_index_input < 0)
                {
                    send_dbg_head(ch_index, cfg->rng_nfft, chirpNum);
                }
                else
                {
                    send_dbg_head(ch_index_input, cfg->rng_nfft, chirpNum);
                }

                /* 发送数据 */
                for (vel_index = 0; vel_index < chirpNum; vel_index++)
                {
                    for (rng_index = 0; rng_index < cfg->rng_nfft / 2; rng_index++)
                    {
                        if (ch_index_input < 0)
                        {
                            fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, vel_index, bpm_index_input); // print all 4 channels
                        }
                        else
                        {
                            fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index_input, rng_index, vel_index, bpm_index_input); // print specified channel
                        }
                        data_buf[data_idx++] = fft_mem;
                        if (data_idx >= 7)
                        {
                            data_idx = 0;
                            /*完整的7字数据 ， 组包发送*/
                            send_dbg_data(data_buf, 7, 0);
                        }
                    }
                }
                /* 不完整的一组数据 ， 单独处理发送*/
                if (data_idx)
                {
                    send_dbg_data(data_buf, data_idx, 0);
                    data_idx = 0;
                }
                /* 发送尾数据包 */
                send_dbg_end();
            }
            cfg_mode = 0;
            MDELAY(1);
            baseband_switch_mem_access(bb_hw, old);
        }
    }

    BB_WRITE_REG(bb_hw, SYS_BNK_ACT, old_bnk);
    BB_WRITE_REG(bb_hw, SYS_ENABLE, old_status_en);
    baseband_switch_buf_store(bb_hw, old_buf_store);

//    newParam_flag_set(NEW_PARAM_SET);

    return 0;
}

extern void convert(uint8_t *ary);

