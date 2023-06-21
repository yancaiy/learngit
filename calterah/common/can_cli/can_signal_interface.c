#include "can_signal_interface.h"
#include "baseband.h"
#include "semphr.h"
#include "dbg_gpio_reg.h"
#include "baseband_task.h"
#include "sensor_config.h"
#include "embARC.h"
#include "can_hal.h"
#include "can_cli.h"
#include "can_obj.h"
#include "baseband_reg.h"
#include "dev_can.h"
#include "baseband_cli.h"
#include "radio_ctrl.h"
#include "apb_lvds.h"
#include "baseband.h"
#include "baseband_dpc.h"
#include "baseband_cli.h"
#include "baseband_task.h"
#include "track_cli.h"
#include "cascade.h"
#include "baseband_hw.h"
#include "baseband_cas.h"
#include "track.h"
#include "string.h"

extern SemaphoreHandle_t mutex_frame_count;
extern int32_t frame_count;
extern int32_t initial_flag;
extern SemaphoreHandle_t mutex_initial_flag;

/* when output data via CAN/CANFD, controlling the bk and ak frame ID 0/1/2 data output
 * default is nothing output, need to choose output bk or ak data by uart command(track new_format)*/
bool output_bk_id0 = false;
bool output_bk_id1 = false;
bool output_bk_id2 = false;    /* Reserved */

bool output_ak_id0 = false;
bool output_ak_id1 = false;
bool output_ak_id2 = false;    /* Reserved */

#define TCP_SEG_ARRAY_LEN          4

#define SINGLE_TONE_MODE           0b0
#define NORMAL_MODE                0b1

#define START                      0b1
#define NO_STREAM                  0b00            /* No using the stream param */
#define STREAM_ON                  0b01
#define STREAM_OFF                 0b10
#define SAMPLE_NULL                0b000
#define SAMPLE_ADC                 0b001
#define SAMPLE_FFT1D               0b010
#define SAMPLE_FFT2D               0b011
#define SAMPLE_DBG                 0b100
#define SAMPLE_SMK                 0b101

#define SYS_ENA(MASK, var) (var << SYS_ENABLE_##MASK##_SHIFT)
#define BB_WRITE_REG(bb_hw, RN, val) baseband_write_reg(bb_hw, BB_REG_##RN, val)

/* The number of arrays required for a CAN frame*/
#define FRAME_ARRAY_NUM                 (4)
#define TCP_SEG_NUM                      4
#define RX_ANT_NUM                       4



static uint32_t track_data[FRAME_ARRAY_NUM] = {0};

uint32_t tcp_seg_data[TCP_SEG_NUM] = {0};

extern volatile enum OBJECT_OUTPUT new_format;

static int32_t can_ota_handler(uint8_t *data, uint32_t len);
static int32_t can_ota_comm_handshake(uint8_t *data, uint32_t len);
static char *reverse(char *c);
static char *self_itoa(uint32_t n);

int32_t can_scan_signal(uint8_t *data, uint32_t len)
{
#ifdef CHIP_CASCADE
	    char cascade_cmd[50];
	    char *frame_len;
	    char scan_cmd[] = "scan start ";
	    char stream_cmd[] = " stream_on ";
	    char adc_cmd[] = "adc", fft1d_cmd[] = "fft1d", fft2d_cmd[] = "fft2d", dbgsam_cmd[] = "dbg_sam";
#endif
        set_track_cfg(2);
        int32_t param1, param2, param3, param4,param5;
        can_config_t can_param;
        /* getting the output data config of bk and ak */
        output_bk_id0 = (data[4] & 0x1);                         /* ID 0x400 */
        output_bk_id1 = ((data[4] >> 1) & 0x1);                  /* ID 0x401 */
        output_bk_id2 = ((data[4] >> 2) & 0x1);                  /* ID 0x402 */
        output_ak_id0 = ((data[4] >> 3) & 0x1);                  /* ID 0x500 */
        output_ak_id1 = ((data[4] >> 4) & 0x1);                  /* ID 0x501 */
        output_ak_id2 = ((data[4] >> 5) & 0x1);                  /* ID 0x502 */

        param1 = data[0] & 0x1;                                  /* Start/Stop:1/0 */
        param2 = (data[1] | (data[2] << 8) | (data[3] << 16));   /* Number of Frame */
        param3 = (data[0] >> 1) & 0x3;                           /* Stream_on/Stream_off:0b01/0b10 */
        param4 = (data[0] >> 3) & 0x7;                           /* Sample_Type: adc/fft1/fft2/cfar/bpm */
        param5 = data[5] & 0x1; /* can_tx_polling mode/can_tx_int mode : 0/1 */

        can_get_config(CAN_0_ID, &can_param);
        if(can_param.data_tx_mode != param5){
            can_param.data_tx_mode = param5;
            can_set_config(CAN_0_ID,&can_param);
        }

        if(param5 == DEV_XFER_POLLING){
            new_format = CAN_POLLING;
        }else{
            new_format = CAN_INT;
        }

        set_baseband_stream_on_dmp_mid(false);
        set_baseband_stream_on_dmp_fnl(false);
        set_baseband_stream_on_fft1d(false);

        EMBARC_PRINTF("can_scan_signal: start:%d\n",param1);

        int32_t count = 0;
        uint8_t dump_src = DBG_SRC_DUMP_W_SYNC;

        if (param1 == START){
                baesband_frame_interleave_cnt_clr();            /* clear recfg count in frame interleaving */
                set_scan_stop_flag(false);
                set_stream_on_en(false);
#ifdef CHIP_CASCADE
                strcpy(cascade_cmd, scan_cmd);
#endif
                if (param2 == 0) {
                    count = -1;
                } else {
                    count = param2;
#ifdef CHIP_CASCADE
                    frame_len = self_itoa(param2);
                    strcat(cascade_cmd, frame_len);
#endif
                }

        } else {
#ifdef CHIP_CASCADE
                if (chip_cascade_status() == CHIP_CASCADE_MASTER)
                        set_scan_stop_flag(true);
#else
                baseband_scan_stop();                          /* scan stop */
#endif
                return pdFALSE;
        }

        if (param3 == STREAM_ON) {
                set_stream_on_en(true);
#ifdef CHIP_CASCADE
                strcat(cascade_cmd, stream_cmd);
#endif
                if (param4 == SAMPLE_ADC) {
#ifdef FUNC_SAFETY
                        /* dynamic sample adc flag, used to stop SM11 and SM12 under functional-safety mode */
                        sample_adc_running_flag = true;
#endif
                        set_baseband_stream_on_dmp_mid(true);
                        set_baseband_stream_on_fft1d(true);
                        baseband_switch_buf_store(NULL, SYS_BUF_STORE_ADC);
#ifdef CHIP_CASCADE
                        strcat(cascade_cmd, adc_cmd);
#endif
                } else if (param4 == SAMPLE_FFT1D) {
                        set_baseband_stream_on_dmp_mid(true);
                        baseband_switch_buf_store(NULL, SYS_BUF_STORE_FFT);
#ifdef CHIP_CASCADE
                        strcat(cascade_cmd, fft1d_cmd);
#endif
                } else if (param4 == SAMPLE_FFT2D) {
                        set_baseband_stream_on_dmp_fnl(true);
#ifdef CHIP_CASCADE
                        strcat(cascade_cmd, fft2d_cmd);
#endif
                } else if (param4 == SAMPLE_DBG) {
                        dump_src = DBG_SRC_SAM; /* config sample debug data to GPIO */
                        bb_clk_switch(); /* debug data has no buffer, bb clock should be switched to dbgbus clock */
#ifdef CHIP_CASCADE
                        strcat(cascade_cmd, dbgsam_cmd);
#endif
                } else {
                        set_stream_on_en(false);
                }
        }

#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER){
            baseband_write_cascade_cmd(cascade_cmd);
        }
#endif

        if (get_stream_on_en()) {
            lvds_dump_start(dump_src);
        }


        /* change frame number */
        xSemaphoreTake(mutex_frame_count, portMAX_DELAY);
        frame_count = count;
        xSemaphoreGive(mutex_frame_count);
        return pdFALSE;
}
////
////void track_pkg_can_print(track_t *track)
////{
////        /* variable */
////        uint16_t i = 0;
////        uint32_t bk_cnt = 0;
////        uint32_t ak_cnt = 0;
////        can_config_t can_param;
////
////        void* sys_params = track->radar_params[baseband_get_cur_frame_type()];
////
////        if (track->trk_update_header) {
////            track->trk_update_header(track->trk_data, sys_params);
////        }
////
////
////        /* Check BK flags to decide if report BK number in CAN header frame */
////        if (output_bk_id0 == 0 && output_bk_id1 == 0) {
////            bk_cnt = 0;
////        } else {
////            bk_cnt = track->cdi_pkg.cdi_number;
////        }
////        if (output_ak_id0 == 0 && output_ak_id0 == 0) {
////            ak_cnt = 0;
////        } else {
////            ak_cnt = 0;
////            for (i = 0; i < TRACK_NUM_TRK; i++) {
////                if (track->trk_update_obj_info)
////                {
////                        track->trk_update_obj_info(track->trk_data, sys_params, i);
////                }
////                if (track->output_obj.output) {
////                    ak_cnt ++;
////                }
////            }
////        }
////        /* Head Data - 3 WORD Length : From track_data[0] to track_data[3] */
////        /* track_data[0] - ID 0x300 Byte0 -> Byte3 */
////        /* track_data[1] - ID 0x300 Byte4 -> Byte7 */
////        /* track_data[2] - ID 0x301 Byte0 -> Byte3 */
////        /* track_data[3] - ID 0x301 Byte4 -> Byte7 */
////        track_data[0] = (((uint32_t)((track->output_hdr.frame_int * 100) + ROUNDF) & LOW10BITMASk)
////                        | ((track->output_hdr.frame_id & LOW16BITMASk) << 16));
////        track_data[1] = (((track->output_hdr.frame_id >> 16) & LOW16BITMASk)
////                        | ((bk_cnt & LOW10BITMASk) << 16));
////        track_data[2] = ((ak_cnt & LOW10BITMASk)
////                        | ((track->cdi_pkg.raw_number & LOW10BITMASk) << 16));
////        track_data[3] = 0x0;                              /* Reserved */
////
////        can_get_config(CAN_0_ID, &can_param);
////        if(can_param.can_mode == CAN_FD_MODE){
////                /* If use CAN FD, data length can be 16 Bytes at one time */
////                /* Send a frame data of Head to CAN bus */
////                can_send_data(CAN_0_ID, HEAD_FRAME_ID0, &track_data[0], eDATA_LEN_16);
////        } else {
////                /* If use CAN, data length can only be 8 Bytes at one time */
////                /* Send a frame data of Head to CAN bus */
////                can_send_data(CAN_0_ID, HEAD_FRAME_ID0, &track_data[0], eDATA_LEN_8);
////                can_send_data(CAN_0_ID, HEAD_FRAME_ID1, &track_data[2], eDATA_LEN_8);
////        }
////
////        /* BK Data */
////        for (i = 0; i < track->cdi_pkg.cdi_number; i++) {
////                float tmpS = track->cdi_pkg.cdi[i].raw_z.sig;
////                float tmpN = track->cdi_pkg.cdi[i].raw_z.noi;
////                float SNR = 10*log10f(tmpS/tmpN);
////
////                track_data[0] = ((i & LOW10BITMASk)
////                                | (((uint32_t)((track->cdi_pkg.cdi[i].raw_z.rng * 100) + ROUNDF)) & LOW16BITMASk) << 16);
////                track_data[1] = (((((uint32_t)((track->cdi_pkg.cdi[i].raw_z.rng * 100) + ROUNDF)) >> 16) & LOW16BITMASk)
////                                | ((transform_data(track->cdi_pkg.cdi[i].raw_z.vel) & LOW15BITMASk) << 16));
////                track_data[2] = ((transform_data(SNR) & LOW15BITMASk)
////                                | ((transform_data(track->cdi_pkg.cdi[i].raw_z.ang) & LOW15BITMASk) << 16));
////#if (TRK_CONF_3D == 0)
////                track_data[3] = 0x4000;
////#else
////                track_data[3] = transform_data(track->cdi_pkg.cdi[i].raw_z.ang_elv) & LOW15BITMASk;
////#endif
////
////                if(can_param.can_mode == CAN_FD_MODE){
////                        /* If use CAN FD, data length can be 16 Bytes at one time */
////                        /* Send a frame data of BK to CAN bus */
////                        if (output_bk_id0) {
////                                can_send_data(CAN_0_ID, BK_FRAME_ID0, &track_data[0], eDATA_LEN_16);
////                        }
////                }else{
////                        /* If use CAN, data length can only be 8 Bytes at one time */
////                        /* Send a frame data of BK to CAN bus */
////                        if (output_bk_id0) {
////                                can_send_data(CAN_0_ID, BK_FRAME_ID0, &track_data[0], eDATA_LEN_8);
////                        }
////                        if (output_bk_id1) {
////                                can_send_data(CAN_0_ID, BK_FRAME_ID1, &track_data[2], eDATA_LEN_8);
////                        }
////                }
////
////        }
////
////        /* AK Data */
////        for (i = 0; i < TRACK_NUM_TRK; i++) {
////                if (track->trk_update_obj_info)
////                {
////                        track->trk_update_obj_info(track->trk_data, sys_params, i);
////                }
////                if (track->output_obj.output) {
////                        track_data[0] = ((i & LOW10BITMASk)
////                                        | ((track->output_obj.track_level & LOW2BITMASk) << 10)
////                                        | ((((uint32_t)((track->output_obj.rng  * 100) + ROUNDF)) & LOW16BITMASk) << 16));
////                        track_data[1] = (((((uint32_t)((track->output_obj.rng  * 100) + ROUNDF)) >> 16)& LOW16BITMASk)
////                                        | ((transform_data(track->output_obj.vel) & LOW15BITMASk) << 16));
////                        track_data[2] = ((transform_data(track->output_obj.SNR) & LOW15BITMASk)
////                                                  | (transform_data(track->output_obj.ang) & LOW15BITMASk) << 16);
////#if (TRK_CONF_3D == 0)
////                        track_data[3] = 0x4000;
////#else
////                        track_data[3] = transform_data(track->output_obj.ang_elv) & LOW15BITMASk;
////#endif
////
////                       if(can_param.can_mode == CAN_FD_MODE){
////                               /* If use CAN FD, data length can be 16 Bytes at one time */
////                               /* Send a frame data of AK to CAN bus */
////                               if (output_ak_id0) {
////                                        can_send_data(CAN_0_ID, AK_FRAME_ID0, &track_data[0], eDATA_LEN_16);
////                               }
////                       }else{
////                               /* If use CAN, data length can only be 8 Bytes at one time */
////                               /* Send a frame data of AK to CAN bus */
////                               if (output_ak_id0) {
////                                        can_send_data(CAN_0_ID, AK_FRAME_ID0, &track_data[0], eDATA_LEN_8);
////                               }
////                               if (output_ak_id1) {
////                                        can_send_data(CAN_0_ID, AK_FRAME_ID1, &track_data[2], eDATA_LEN_8);
////                               }
////                       }
////                }
////        }
////}
//
//
//
///*
// * track data output by can intr mode
// * Notes:For can tx intr mode, only support data lenth:16, 32, 64
// */
//void track_pkg_can_int_print(track_t *track)
//{
//    uint32_t i = 0;
//    uint32_t bk_cnt = 0;
//    uint32_t ak_cnt = 0;
//    uint32_t idx = 0;
//    uint32_t size = 0;
//	uint32_t data_len = 0;
//    void *sys_params = track->radar_params[baseband_get_cur_frame_type()];
//    can_config_t can_param;
//
//    /* check if need memset? */
//    if (track->trk_update_header) {
//        track->trk_update_header(track->trk_data, sys_params);
//    }
//
//    /* Check BK flags to decide if report BK number in CAN header frame */
//    if (output_bk_id0 == 0) {
//        bk_cnt = 0;
//    } else {
//        bk_cnt = track->cdi_pkg.cdi_number;
//    }
//    if (output_ak_id0 == 0) {
//        ak_cnt = 0;
//    } else {
//        ak_cnt = 0;
//        for (i = 0; i < TRACK_NUM_TRK; i++) {
//            if (track->trk_update_obj_info) {
//                track->trk_update_obj_info(track->trk_data, sys_params, i);
//            }
//            if (track->output_obj.output) {
//                ak_cnt++;
//            }
//        }
//    }
//
//    /* Head Data - 4 WORD Length : From track_data[0] to track_data[3] */
//    /* track_data ID 0x600 */
//    trk_hex.data[idx++] = 0xFFEEFFDC;     /* frame hdr magic number */
//    trk_hex.data[idx++] = (((uint32_t)((track->output_hdr.frame_int * 100) + ROUNDF) & LOW10BITMASk)
//                          | ((track->output_hdr.frame_id & LOW16BITMASk) << 16U)
//                          | (((output_bk_id0) | (output_ak_id0 << 1U)) << 10U));
//    trk_hex.data[idx++] = (((track->output_hdr.frame_id >> 16U) & LOW16BITMASk)
//                          | ((bk_cnt & LOW10BITMASk) << 16U));
//    trk_hex.data[idx++] = ((ak_cnt & LOW10BITMASk)
//                          | ((track->cdi_pkg.raw_number & LOW10BITMASk) << 16U));
//
//
//    if (output_bk_id0) {
//
//        /* BK Data */
//        for (i = 0; i < track->cdi_pkg.cdi_number; i++) {
//            float tmpS = track->cdi_pkg.cdi[i].raw_z.sig;
//            float tmpN = track->cdi_pkg.cdi[i].raw_z.noi;
//            float SNR = 10*log10f(tmpS/tmpN);
//
//            trk_hex.data[idx++] = ((i & LOW10BITMASk)
//                                  | (((uint32_t)((track->cdi_pkg.cdi[i].raw_z.rng * 100) + ROUNDF)) & LOW16BITMASk) << 16);
//            trk_hex.data[idx++] = (((((uint32_t)((track->cdi_pkg.cdi[i].raw_z.rng * 100) + ROUNDF)) >> 16) & LOW16BITMASk)
//                                  | ((transform_data(track->cdi_pkg.cdi[i].raw_z.vel) & LOW15BITMASk) << 16));
//            trk_hex.data[idx++] = ((transform_data(SNR) & LOW15BITMASk)
//                                  | ((transform_data(track->cdi_pkg.cdi[i].raw_z.ang) & LOW15BITMASk) << 16));
//#if (TRK_CONF_3D == 0)
//            trk_hex.data[idx++] = 0x4000;
//#else
//            trk_hex.data[idx++] = transform_data(track->cdi_pkg.cdi[i].raw_z.ang_elv) & LOW15BITMASk;
//#endif
//        }
//    }
//
//    if (output_ak_id0) {
//        /* AK Data */
//        for (i = 0; i < TRACK_NUM_TRK; i++) {
//            if (track->trk_update_obj_info) {
//                track->trk_update_obj_info(track->trk_data, sys_params, i);
//            }
//            if (track->output_obj.output) {
//                trk_hex.data[idx++] = ((i & LOW10BITMASk)
//                                      | ((track->output_obj.track_level & LOW2BITMASk) << 10)
//                                      | ((((uint32_t)((track->output_obj.rng  * 100) + ROUNDF)) & LOW16BITMASk) << 16));
//                trk_hex.data[idx++] = (((((uint32_t)((track->output_obj.rng  * 100) + ROUNDF)) >> 16)& LOW16BITMASk)
//                                      | ((transform_data(track->output_obj.vel) & LOW15BITMASk) << 16));
//                trk_hex.data[idx++] = ((transform_data(track->output_obj.SNR) & LOW15BITMASk)
//                                      | (transform_data(track->output_obj.ang) & LOW15BITMASk) << 16);
//#if (TRK_CONF_3D == 0)
//                trk_hex.data[idx++] = 0x4000;
//#else
//                trk_hex.data[idx++] = transform_data(track->output_obj.ang_elv) & LOW15BITMASk;
//#endif
//            }
//        }
//    }
//
//    /* output data align */
//    can_get_config(CAN_0_ID, &can_param);
//    if(can_param.can_mode == CAN_MODE){
//        data_len = CAN_FRAM_BUF_DATA_SIZE;
//    }else{
//        data_len = CAN_FD_FRAM_BUF_DATA_SIZE;
//    }
//    size = ALIGN_SIZE(idx << 2, data_len);
//#ifdef FUNC_SAFETY
//        /* set can send status to CAN_SEND_STATUS_SENDING */
//        set_can_send_status(CAN_SEND_STATUS_SENDING);
//#endif
//    can_send_data(CAN_0_ID, FRAME_ID, trk_hex.data, size);
//}


static int32_t can_ota_handler(uint8_t *data, uint32_t len)
{
        int32_t result = E_OK;

        result = can_ota_comm_handshake(data, len);

        if (E_OK == result) {
                fmcw_radio_reboot_cause_set(ECU_REBOOT_CAN_OTA);

                chip_hw_mdelay(1);

                fmcw_radio_reset();
        }
        return  result;
}

static int32_t can_ota_comm_handshake(uint8_t *data, uint32_t len)
{
        int32_t result = E_OK;
        uint32_t ota_comm_data[2] = {0};

        transfer_word_stream(data, ota_comm_data, len);

        if ((CAN_OTA_COM_MAGIC_NUM == ota_comm_data[0]) && \
            (CAN_OTA_COM_HS_CODE == ota_comm_data[1])) {
                /* If the received "Magic number" and "HS code" are equal to the setting value,
                    return the received value to Host */
                can_send_data(CAN_0_ID, CAN_OTA_ID, ota_comm_data, len);
        } else {
                result = E_NODATA;
        }

        return result;
}

int32_t can_dump_signal(uint8_t *data, uint32_t len)
{
        set_track_cfg(2);
        int32_t param1, param2, param3;
        bool single_tone_en = false;
        bool smoke_test_en = false;

        uint32_t dump_position = 0;
        int32_t nframe = 100;

        uint16_t bb_sys_en = 0;
        bool tx_en = true;
        bool radio_en = true;
        bool cas_sync_en = true;

        baseband_t* bb = baseband_get_rtl_frame_type();  // read back the bank selected in RTL
        sensor_config_t* cfg = (sensor_config_t*)(bb->cfg);
        fmcw_radio_t* radio = &bb->radio;
        baseband_hw_t* bb_hw = &bb->bb_hw;

        param1 = data[0] & 0x1;                                   /* Single-tone/Normal:0/1 */
        param2 = (data[1] | (data[2] << 8) | (data[3] << 16));    /* Number of Frame: LSB data[1] MSB data[2] */
        param3 = (data[0] >> 3) & 0x7;                            /* Sample_Type: adc/fft1d/fft2d/smoke_test:001/010/011/101 */

        if (param1 == SINGLE_TONE_MODE) {
                tx_en = false;
                radio_en = false;
                single_tone_en = true;
        }

        if (param2 == 0) {
            nframe = -1;
        } else {
            nframe = param2;
        }

        if (param3 == SAMPLE_ADC) {
                dump_position = SYS_BUF_STORE_ADC;
                bb_sys_en =  SYS_ENA(SAM    , true)
                            |SYS_ENA(DMP_FNL, true);
        } else if (param3 == SAMPLE_FFT1D) {
                dump_position = SYS_BUF_STORE_FFT;
                bb_sys_en =  SYS_ENA(SAM    , true)
                            |SYS_ENA(DMP_FNL, true);
        } else if (param3 == SAMPLE_FFT2D) {
                dump_position = SYS_BUF_STORE_FFT;
                bb_sys_en =  SYS_ENA(SAM    , true)
                            |SYS_ENA(FFT_2D , true)
                            |SYS_ENA(DMP_FNL, true);
        } else if (param3 == SAMPLE_SMK){
                tx_en = false;
                radio_en = false;
                cas_sync_en = false;
                smoke_test_en = true;
                bb_sys_en = SYS_ENA(DMP_FNL, true);
        }
        else
        {

        }

#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER){
                // Todo: spi send command to slave
        }
#endif

        baesband_frame_interleave_cnt_clr(); // clear recfg count in frame interleaving

        /* lvds start */
        lvds_dump_start(DBG_SRC_DUMP_W_SYNC);

        /* single tone settings */
        if (single_tone_en) {
                fmcw_radio_tx_ch_on(radio, -1, false); /* turn off tx */
                fmcw_radio_single_tone(radio, cfg->fmcw_startfreq, true); /* enable radio single tone of fmcw_startfreq */
                MDELAY(1); /* for FMCW settled */
                BB_WRITE_REG(bb_hw, SAM_FORCE, SAM_FORCE_ENABLE);
        }

        /* smoke test data pattern settings */
        if (smoke_test_en) /* write data pattern to memory */
        {
                baseband_datdump_smoke_test(bb_hw);
        }
        
        /* timer start */
        track_start(bb->track);

        while(1) {
                if ((nframe != 0) && (track_is_ready(bb->track))) {
                        track_lock(bb->track);

                        /* align bank of frame type */
                        bb = baseband_frame_interleave_recfg(); /* reconfigure the frame pattern */
                        cfg = (sensor_config_t*)(bb->cfg);
                        bb_hw = &bb->bb_hw;

                        /* baseband dump init */
                        uint32_t old_buf_store = baseband_switch_buf_store(bb_hw, dump_position);
                        uint8_t old_bnk = BB_READ_REG(bb_hw, FDB_SYS_BNK_ACT); /* read back the bank selected in RTl */
                        uint16_t old_status_en = BB_READ_REG(bb_hw, SYS_ENABLE);
                        /* Clear event bit before bb start */
                        uint32_t event_bits = baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
                        if( event_bits != E_OK)
                        {
                                EMBARC_PRINTF("Event bit set to %d before start @%s", event_bits, __func__);
                        }
                        /* baseband start */
                        baseband_start_with_params(bb, radio_en, tx_en, bb_sys_en, cas_sync_en, BB_IRQ_ENABLE_ALL, false);

                        /* Wait BB done */
                        BASEBAND_ASSERT(baseband_wait_bb_done(INT_MODE, DEFAULT_TIMOUT) == E_OK);

                        /* restore baseband status */
                        /* the following 3 lines should be inside the while loop, as bank index will change in multi mode */
                        BB_WRITE_REG(bb_hw, SYS_BNK_ACT, old_bnk);
                        BB_WRITE_REG(bb_hw, SYS_ENABLE, old_status_en);
                        baseband_switch_buf_store(bb_hw, old_buf_store);

                        EMBARC_PRINTF("nframe: %d\n", nframe);
                        nframe--;
                } else if( nframe == 0 ) {
                        break;
                } else {
                        taskYIELD();
                }
        }
        /* dump finish */
        lvds_dump_stop();
        track_stop(bb->track);

        /* single tone settings restore */
        if (single_tone_en == true) {
                BB_WRITE_REG(&bb->bb_hw, SAM_FORCE, SAM_FORCE_DISABLE);
                baseband_hw_reset_after_force(&bb->bb_hw);
#if INTER_FRAME_POWER_SAVE == 0 /* when power save enabled, restore will be run when next "baseband_start_with_params" */
                fmcw_radio_tx_restore(radio);
#endif
        }

        return pdFALSE;
}

static int32_t can_hil_signal(uint8_t *data, uint32_t len)
{
        set_track_cfg(2);
        int32_t param1, param2, param3, param4;

        param1 = data[0] & 0x1;                                 /* Start/Stop:1/0 */
        param2 = (data[0] >> 1) & 0x1;                          /* hil_gpio/ahb:1/0 */
        param3 = (data[0] >> 2) & 0x7;                          /* fft1/fft2/null mode::10/11/00 */
        param4 = (data[1] | (data[2] << 8) | (data[3] << 16));  /* Number of Frame */

        /* getting the output data config of bk and ak */
        output_bk_id0 = (data[4] & 0x1);                         /* ID 0x400 */
        output_bk_id1 = ((data[4] >> 1) & 0x1);                  /* ID 0x401 */
        output_bk_id2 = ((data[4] >> 2) & 0x1);                  /* ID 0x402 */
        output_ak_id0 = ((data[4] >> 3) & 0x1);                  /* ID 0x500 */
        output_ak_id1 = ((data[4] >> 4) & 0x1);                  /* ID 0x501 */
        output_ak_id2 = ((data[4] >> 5) & 0x1);                  /* ID 0x502 */

        baseband_t *ptr_bb = baseband_get_cur_bb();
        baseband_hw_t *bb_hw = &ptr_bb->bb_hw;
        bool hil_start = false;
        bool hil_mux = false;
        uint8_t dmp_mux = 0;

        if (param1 == START) {
                hil_start = true;
        } else {
                baseband_t* p_bb = baseband_get_rtl_frame_type();
                baseband_stop(p_bb);
                return pdFALSE;
        }

        if (param2 == HIL_GPIO) {
                hil_mux = true;
        } else {
                /* AHB mode */
                hil_mux = false;
        }

        /* hil from ahb is not necessary to dump data */
        if (param3 == SAMPLE_FFT1D) {
                dmp_mux = 1;
        } else if (param3 == SAMPLE_FFT2D) {
                dmp_mux = 2;
        } else if (param3 == SAMPLE_NULL) {
                /* NULL mode */
                dmp_mux = 0;
        }
        else
        {

        }

        /* HIL on */
        if (true == hil_start) {
                set_stream_on_en(true);
                /* param4 is for frame number */
                baseband_hil_on(bb_hw, hil_mux, dmp_mux, param4);
        }

        return pdFALSE;
}

static uint16_t get_chirp_nmb(sensor_config_t* cfg)
{
        uint16_t tx_ch_nmb = 0;

        /* calculate tx number */
        uint32_t tx_groups_value = cfg->tx_groups[0] | cfg->tx_groups[1] |\
                                   cfg->tx_groups[2] | cfg->tx_groups[3];

        if ((tx_groups_value & 0xF000) > 0) {
                tx_ch_nmb = 4;
        } else if ((tx_groups_value & 0xF00) > 0) {
                tx_ch_nmb = 3;
        } else if ((tx_groups_value & 0xF0) > 0) {
                tx_ch_nmb = 2;
        } else if ((tx_groups_value & 0xF) > 0) {
                tx_ch_nmb = 1;
        }
        else
        {

        }

        if (cfg->anti_velamb_en) {
                tx_ch_nmb++;
        }
        return tx_ch_nmb;
}

static int32_t can_sensor_cfg_signal(uint8_t *data, uint32_t len)
{
        set_track_cfg(2);
        uint16_t tx_ch_nmb = 0;
        uint16_t chirp_nmb = 0;
        uint16_t skip_nmb = 0;
        uint16_t sample_nmb = 0;
        uint8_t rx_ant_num = 0;
        uint8_t k = 0;

        /* receive the data from CAN bus */
        uint8_t type          = data[0];            /* TCP seg type */
        uint8_t hil_ena       = data[1];
        uint8_t dbg_src       = data[2];
        uint8_t dbg_lvds_type = data[3];
        uint32_t frame_nmb    = data[4] | (data[5] << 8) | (data[6] << 16);
        uint8_t hil_cfg_mode  = data[7];

        EMBARC_PRINTF("can_sensor_cfg_signal is from GUI\n");

        /* tansmit the data of tcp seg to CAN bus */
        /* TCP header, It takes 4 bytes. */
        rx_ant_num = RX_ANT_NUM; //alps rx ant num is fix to be 4

        for(k=0;k<TCP_SEG_NUM;k++){
            tcp_seg_data[k] = 0;
        }
        tcp_seg_data[0] = type | ((TCP_CTRL_PACKAGE_MAGICNUMBER & 0xFFFFFF) << 8);
        tcp_seg_data[1] = (NUM_FRAME_TYPE & 0xFF) | (hil_cfg_mode << 8) | (rx_ant_num << 16);

        can_send_data(CAN_0_ID, CAN_TCP_SEG_ID, &tcp_seg_data[0], eDATA_LEN_8);

        /* TCP sensor configuartion, It takes 15 bytes each config. */
        for (uint32_t i = 0; i < 4; i++) {
                sensor_config_t* cfg = sensor_config_get_config(i);

                if (NULL != cfg) {
                        tx_ch_nmb = get_chirp_nmb(cfg);
                        chirp_nmb = cfg-> vel_nfft * tx_ch_nmb;

                        /* skip number */
                        skip_nmb = cfg->fmcw_startfreq * cfg->adc_freq / cfg->dec_factor;

                        /* sample number */
                        sample_nmb = cfg->rng_nfft;
                }

                /* default value 1600 */
                uint16_t chirp_len = 1600;

                /* velocity ambiguity enable */
                uint8_t de_vel_amb_ena = 0;

                /* velocity ambiguity chirp number */
                uint8_t de_vel_amb_nmb = 0;

                tcp_seg_data[0] = hil_ena | (dbg_src << 8) | (dbg_lvds_type << 16) |\
                                  ((frame_nmb & 0xFF) << 24);
                tcp_seg_data[1] = ((frame_nmb >> 8) & 0xFF) | (((frame_nmb >> 16) & 0xFF) << 8) |\
                                  ((chirp_nmb & 0xFF) << 16) | (((chirp_nmb >> 8) & 0xFF) << 24);
                tcp_seg_data[2] = (chirp_len & 0xFF) | (((chirp_len >> 8) & 0xFF) << 8) |\
                                  ((skip_nmb & 0xFF) << 16) | (((skip_nmb >> 8) & 0xFF) << 24);
                tcp_seg_data[3] = (sample_nmb & 0xFF) | (((sample_nmb >> 8) & 0xFF) << 8) |\
                                  (de_vel_amb_ena << 16) | (de_vel_amb_nmb << 24);

                can_send_data(CAN_0_ID, CAN_TCP_SEG_ID, &tcp_seg_data[0], eDATA_LEN_8);
                can_send_data(CAN_0_ID, CAN_TCP_SEG_ID, &tcp_seg_data[2], eDATA_LEN_8);
        }

        return pdFALSE;
}

static char *reverse(char *c)
{
	char temp;
	char *p = c;
	char *q = c;
	while (*q)
	{
		++q;
    }
	q--;

	while (q > p) {
		temp = *p;
		*p++ = *q;
		*q-- = temp;
	}

	return c;
}
static char *self_itoa(uint32_t n)
{
	static char s[10];
	uint8_t i = 0;

	do
	{
		s[i++] = n % 10 + '0';
		n = n / 10;
	} while (n > 0);

	s[i] = '\0';

	return reverse(s);
}

//int32_t can_cli_commands(void)
//{
//        int32_t result = E_OK;
//
//        /* register scan command */
//        result = can_cli_register(SCAN_FRAME_ID, can_scan_signal);
//        /* register CAN OTA start command */
//        result = can_cli_register(CAN_OTA_ID, can_ota_handler);
//        /* register CAN dump command*/
//        result = can_cli_register(CAN_DUMP_ID, can_dump_signal);
//        /* register CAN sensor configuration command*/
//        result = can_cli_register(CAN_SENSOR_CFG_ID, can_sensor_cfg_signal);
//        /* register CAN HIL command*/
//        result = can_cli_register(CAN_HIL_ID, can_hil_signal);
//
//        return result;
//}

