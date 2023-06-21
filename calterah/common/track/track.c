/*
   This file implements interface between tracking and firmware.
   Please DONOT include any algorithm specific data, info and defs here!!!
*/

/*--- INCLUDE ------------------------*/
#include <math.h>
#include <string.h>
#include "calterah_math.h"
#include "track.h"
#include "track_cli.h"
#include "ekf_track.h"
#include "baseband.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "sensor_config.h"
#include "console.h"
#include "can_signal_interface.h"
#include "track_cli.h"
#include "vel_deamb_MF.h"
#include "cascade.h"
#include "bb_flow.h"
#include "baseband_n2d.h"
#include "can_hal.h"
#include "baseband_n2d.h"
#include "target_proc.h"
#include "radardsp.h"
#include "app_dut.h"
#include "rsp_datapath.h"


#ifndef M_PI
#define M_PI 3.1415926535f
#endif

/* DEG2RAD = pi/180 */
#define DEG2RAD 0.017453292519943295f

/* RAD2DEG = 180/pi */
#define RAD2DEG 57.295779513082323

static track_t track_ctrl;
static uint8_t current_frame_type = 0;

/*--- DEFINE -------------------------*/
#define MAX_Q_IDX 15
#define MAX_Q_NUM 32
#define BB_READ_REG(bb_hw, RN) baseband_read_reg(bb_hw, BB_REG_##RN)

/*--- DECLARATION --------------------*/
static void track_update_time  (track_t *track);    /* update frame time */
static bool track_ready = true;
static xTimerHandle xTimerTrack;

trk_t trk_hex;
extern radarDspParam_t gDspParam;

extern int8_t get_track_cfg(void);

track_t* track_get_track_ctrl()
{
        return &track_ctrl;
}

/* FreeRTOS timer callback */
void vTimerTrackCallback(xTimerHandle xTimer)
{
        track_ready = true;
}
/* init */
void track_init(track_t *track)
{
        sensor_config_t *cfg = sensor_config_get_cur_cfg();

//#ifdef CHIP_CASCADE
//        if (chip_cascade_status() == CHIP_CASCADE_MASTER)
//                install_ekf_track(track);
//#else
//        install_ekf_track(track);
//#endif

        track_ready = true;
        xTimerTrack = xTimerCreate("for track", pdMS_TO_TICKS(1000 / cfg->track_fps), pdTRUE, (void *)0, vTimerTrackCallback);
//        track_cmd_register();
//        init_track_data(MAX_ARRAY_NUM);
}

void track_init_sub(track_t *track, void *ctx)
{
        baseband_t* bb = (baseband_t *)ctx;
        int index = bb->bb_hw.frame_type_id;
        track->radar_params[index] = &bb->sys_params;
}

void track_pre_start(track_t *track)
{
        void* sys_params = track->radar_params[baseband_get_cur_frame_type()];
        if (track->trk_init)
        {
            track->trk_init(track->trk_data, sys_params);
        }
}

bool track_is_ready(track_t *track)
{
        return track_ready;
}
void track_lock(track_t *track)
{
        track_ready = false;
}


/* start */
void track_start(track_t *track)
{
        if(xTimerTrack != NULL)
        {
            xTimerStart(xTimerTrack, 0);
        }
}

/* stop */
void track_stop(track_t *track)
{
        xTimerStop(xTimerTrack, 0);
        if (track->trk_stop)
        {
            track->trk_stop(track->trk_data, NULL);
        }
        track_ready = true;
}

static bool is_obj(const radar_sys_params_t *sys_params, bool valid_flag, float angle, uint32_t sig, bool valid_flag_e, float angle_e, uint32_t sig_e)
{
        if (!valid_flag)
        {
                return false;
        }
        else if ((sig & 0x1f) > 0x1f)
        {
            return false;
        }
        else if ((angle < sys_params->trk_fov_az_left) || (angle > sys_params->trk_fov_az_right))
        {
            return false;
        }
        else if (!valid_flag_e)
        {
            return false;
        }
        else if ((sig_e & 0x1f) > 0x1f)
        {
                return false;
        }
        else if ((angle_e < sys_params->trk_fov_ev_down) || (angle_e > sys_params->trk_fov_ev_up))
        {
            return false;
        }
        else
        {
            return true;
        }
}

//这个函数是为了处理alps样品只能处理32bit的变量操作，之前直接结构体赋值出现系统崩溃现象，所以这里直接每一个幅值
static void copyTarget(Target_Thres_t *des, Target_Thres_t *src)
{
#if 1
    des->angle = src->angle;
    des->heighAngle = src->heighAngle;

    des->range = src->range;
    des->rangeBin = src->rangeBin;
    des->realVel = src->realVel;
    des->threshold = src->threshold;
    des->isUsing = src->isUsing;
    des->valid = src->valid;
    des->velocity = src->velocity;
    des->velocityBin = src->velocityBin;
    des->doaMag = src->doaMag;
    des->fftMag = src->fftMag;
    des->angleNum = 2;
    src->angleNum = 2;
#else
    memcpy((u8 *)des, (u8 *)src, sizeof(Target_Thres_t));
#endif
}

static void obj_check(const radar_sys_params_t *sys_params,
                      bool ang_vld,
                      uint32_t ang_idx,
                      uint32_t u_ang_acc,
                      bool acc_ang_flag,
                      uint32_t sig,
                      bool ang_vld_e,     /*hw elevated angle valid indicate*/
                      uint32_t ang_idx_e, /*hw elevated angle index*/
                      uint32_t u_ang_acc_e, /*hw elevated angle acc value*/
                      uint32_t sig_e,  /*hw elevated angle sig*/
                      bool elv_flag,   /*control if output elevated angle*/
                      int* p_j,
                      Target_Thres_t *cdi,
                      bool is_dml_flag)
{
        int32_t hw_ang = 0, hw_ang_elv = 0;
        float ang_acc = 0.0;
        float angle_cdi = 0.0, angle_cdi_elv = 0.0;
        uint32_t hw_sig = 0;
        float sig_cdi = 0.0;
        #if TRK_CONF_3D
                float sig_cdi_elv = 0.0;
        #endif

        if (*p_j >= TRACK_NUM_CDI){
                return;
        }

        hw_ang = ang_idx;
        ang_acc = fx_to_float(u_ang_acc, 4, 0, true);
        ang_acc = acc_ang_flag ? (ang_acc + (float)hw_ang) : (float)hw_ang;

        if (is_dml_flag) {
                if (u_ang_acc != 0 && sys_params->doa_npoint[0] > 512){
                        hw_ang += 512;
                }
                /* Type(asin_v):double -> float, asin -> asinf. Processing time optimization */
                float asin_v = sys_params->dml_sin_az_left + hw_ang * sys_params->dml_sin_az_step;
                float ang = asinf(asin_v) * RAD2DEG;
                angle_cdi = ang;
        } else {
                angle_cdi = sys_params->bfm_az_left + ang_acc * sys_params->az_delta_deg ;
        }

        hw_sig = sig;
        sig_cdi = fl_to_float(hw_sig, 15, 1, false, 5, false);

        if (elv_flag) {
                hw_ang_elv = ang_idx_e;
                ang_acc = fx_to_float(u_ang_acc_e, 4, 0, true);
                ang_acc = acc_ang_flag ? (ang_acc + (float)hw_ang_elv) : (float)hw_ang_elv;

                if (is_dml_flag) {
                        if (u_ang_acc_e != 0 && sys_params->doa_npoint[1] > 512) {
                            hw_ang_elv += 512;
                        }
                        /* Type(asin_v):double -> float, asin -> asinf. Processing time optimization */
                        float asin_v = sys_params->dml_sin_ev_down + hw_ang_elv * sys_params->dml_sin_ev_step;
                        float ang = asinf(asin_v) * RAD2DEG;
                        angle_cdi_elv = ang;
                } else {
                        angle_cdi_elv = sys_params->bfm_ev_down + ang_acc * sys_params->ev_delta_deg;
                }

#if TRK_CONF_3D
                 /* Type(tmp_sin_theta):double -> float, asin -> asinf. Processing time optimization */
                float tmp_sin_theta = sinf(angle_cdi * DEG2RAD) / cosf(angle_cdi_elv * DEG2RAD);
                if (tmp_sin_theta <= 1 && tmp_sin_theta >= -1) {
                    angle_cdi = asinf(tmp_sin_theta) * RAD2DEG;
                }
                sig_cdi_elv = fl_to_float(sig_e, 15, 1, false, 5, false);
#endif
        } else {
            sig_e = 0xF ;   // avoid unreasonable elevated results in memory to effect the object number
        }

        if (is_obj(sys_params, ang_vld, angle_cdi, sig, ang_vld_e, angle_cdi_elv, sig_e)) {
                int num = *p_j;
                copyTarget(&(cdi[num]), &(cdi[num - 1]));
                if (DUTGetTestState() == DUT_MODE_CLOSE)
                {
                    //根据具体的应用转换角度
                    #if (ANGLE_INVERT_ENABLE==1)
                    cdi[num].angle = 0.0 - angle_cdi;
                    #else
                    cdi[num].angle = angle_cdi;
                    #endif
                }
                else
                {
                    //工厂模式使用原始的点值，不用转换
                }
                cdi[num].angle = angle_cdi;
                cdi[num].doaMag = sig_cdi;
#if TRK_CONF_3D
                cdi[num].heighAngle = -32;
                cdi[num].sig_elv = sig_cdi_elv;
#endif
                *p_j = num + 1;
        }
}

void track_pre_filter(const radar_sys_params_t *sys_params,
                      doa_info_t *doa,
                      int* p_j,
                      Target_Thres_t *cdi,
                      bool acc_ang_flag,
                      bool elv_flag,
                      bool is_dml_flag)
{
        // check multi-obj 1
        // elevated multi-obj 0
        obj_check(sys_params, doa[0].ang_vld_1, doa[0].ang_idx_1, doa[0].ang_acc_1, acc_ang_flag, doa[0].sig_1,
                  true, doa[1].ang_idx_2, doa[1].ang_acc_2, doa[1].sig_2, elv_flag, p_j, cdi, is_dml_flag);
        // elevated multi-obj 1
        if (elv_flag) {
                obj_check(sys_params, doa[0].ang_vld_1, doa[0].ang_idx_1, doa[0].ang_acc_1, acc_ang_flag, doa[0].sig_1,
                          doa[1].ang_vld_3, doa[1].ang_idx_3, doa[1].ang_acc_3, doa[1].sig_3, elv_flag, p_j, cdi, is_dml_flag);
        }

        // check multi-obj 2
        //elevated multi-obj 0
        obj_check(sys_params, doa[0].ang_vld_2, doa[0].ang_idx_2, doa[0].ang_acc_2, acc_ang_flag, doa[0].sig_2,
                  true, doa[2].ang_idx_0, doa[2].ang_acc_0, doa[2].sig_0, elv_flag, p_j, cdi, is_dml_flag);
        //elevated multi-obj 1
        if (elv_flag) {
                obj_check(sys_params, doa[0].ang_vld_2, doa[0].ang_idx_2, doa[0].ang_acc_2, acc_ang_flag, doa[0].sig_2,
                          doa[2].ang_vld_1, doa[2].ang_idx_1, doa[2].ang_acc_1, doa[2].sig_1, elv_flag, p_j, cdi, is_dml_flag);
        }

        // check multi-obj 3
        //elevated multi-obj 0
        obj_check(sys_params, doa[0].ang_vld_3, doa[0].ang_idx_3, doa[0].ang_acc_3, acc_ang_flag, doa[0].sig_3,
                  true, doa[2].ang_idx_2, doa[2].ang_acc_2, doa[2].sig_2, elv_flag, p_j, cdi, is_dml_flag);
        //elevated multi-obj 1
        if (elv_flag) {
                obj_check(sys_params, doa[0].ang_vld_3, doa[0].ang_idx_3, doa[0].ang_acc_3, acc_ang_flag, doa[0].sig_3,
                          doa[2].ang_vld_3, doa[2].ang_idx_3, doa[2].ang_acc_3, doa[2].sig_3, elv_flag, p_j, cdi, is_dml_flag);
        }
}

/* read from hardware */
void track_read(track_t *track)
{
        /* varibles */
        int obj_num;
        int i, j;
        int group_num = 1;

        /* CT varibles*/
        int32_t tx_profile = 0; // 天线参数
        int32_t chirp_type = 0; // 波形参数
        static int32_t tx_mask_flag = 0;
        float globalNoise;

        /* special varibles */
        uint8_t track_frame_type = baseband_get_cur_frame_type();
        current_frame_type = track_frame_type;
        radar_sys_params_t* sys_params = track->radar_params[track_frame_type];
        baseband_hw_t *bb_hw = &(CONTAINER_OF(sys_params, baseband_t, sys_params)->bb_hw);
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(bb_hw, baseband_t, bb_hw)->cfg;
        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_RLT);
        bool elv_flag = false;         //indicate if combined mode are used to calculate elevated angle
        bool is_dml_flag = (cfg->doa_method == 2);

        /* get memory offset */
        uint32_t mem_rlt_offset = 0;
        if ((BB_READ_REG(bb_hw, CFR_SIZE_OBJ)) < 256) {/* mem_rlt will be splited to 4 banks when cfar size less than 256 */
                mem_rlt_offset = track_frame_type * (1 << SYS_SIZE_OBJ_WD_BNK) * RESULT_SIZE;
        }

        //FIX ME if 2D DoA mode is combined 2d or single shot mode, obj_info_t may be need some change
        volatile obj_info_t *obj_info = (volatile obj_info_t *)(BB_MEM_BASEADDR + mem_rlt_offset);
        volatile track_cdi_t *track_cdi = (volatile track_cdi_t *)track->cdi_pkg.cdi;

        bool acc_rng_flag = cfg->acc_rng_hw;
        bool acc_vel_flag = cfg->acc_vel_hw;
        bool acc_ang_flag = cfg->acc_angle_hw;

        /* get object number */
#ifdef CHIP_CASCADE
        obj_num = BB_READ_REG(bb_hw, DOA_NUMB_OBJ);
#else
        obj_num = BB_READ_REG(bb_hw, CFR_NUMB_OBJ);
#endif

        if( obj_num > TRACK_NUM_CDI ) {
                obj_num = TRACK_NUM_CDI;
        }
        if(cfg->doa_mode != 2){
                group_num = BB_READ_REG(bb_hw, DOA_NUMB_GRP) + 1;
        }
        if (cfg->doa_mode == 1){
                elv_flag = true;
        }
		
		DSPConfigSet(track_frame_type, &tx_profile, &chirp_type, &tx_mask_flag);
        /* CT End */

        globalNoise = getGlobalNoise(bb_hw, cfg->nvarray, cfg->rng_nfft, cfg->vel_nfft);
        // 判断当前时刻的底噪值是否比最小底噪值高3db，如果高于3db，则输出最小底噪值加3db。
        if(globalNoise > gDspParam.minimumNoise[tx_profile] + 3)
        {
            globalNoise = gDspParam.minimumNoise[tx_profile] + 3;
        }
        else if(globalNoise < gDspParam.minimumNoise[tx_profile])
        {
            gDspParam.minimumNoise[tx_profile] = globalNoise;
        }

        EMBARC_PRINTF("----- track_read(), obj_num:%x -----\n", obj_num);

        track->output_hdr.frame_type = current_frame_type;

        /* read data into cdi_pkg */
        for (i = 0, j = 0; i < obj_num && j < TRACK_NUM_CDI; i++) {

                /* read index */
                int32_t hw_rng = obj_info[i].rng_idx;
                int32_t hw_vel = obj_info[i].vel_idx;
                int32_t q_idx  = obj_info[i].amb_idx;
                int32_t hw_ang = obj_info[i].doa[0].ang_idx_0;
                doa_info_t *doa_info = (doa_info_t *)obj_info[i].doa;

                float rng_acc = fx_to_float(obj_info[i].rng_acc, 4, 0, true);
                float vel_acc = fx_to_float(obj_info[i].vel_acc, 4, 0, true);
                float ang_acc = fx_to_float(obj_info[i].doa[0].ang_acc_0, 4, 0, true);
                rng_acc = acc_rng_flag ? (rng_acc + (float)hw_rng) : (float)hw_rng;
                vel_acc = acc_vel_flag ? (vel_acc + (float)hw_vel) : (float)hw_vel;
                ang_acc = acc_ang_flag ? (ang_acc + (float)hw_ang) : (float)hw_ang;

                Bin_Param_t bin_param = getBinParam();
                uint32_t hw_sig = obj_info[i].doa[0].sig_0;
                bool az_angle=false, elv_angle=false, angle_invalid=false;
                /* CT set variable*/
				int hw_vel_0_compen = hw_vel + q_idx * cfg->vel_nfft;
                float raw_vel_0_compen = hw_vel_0_compen * bin_param.velocity_bin;

                gTargets[tx_profile][chirp_type][j].firstvelocity = raw_vel_0_compen;
                gTargets[tx_profile][chirp_type][j].valid = 1;
                gTargets[tx_profile][chirp_type][j].isUsing = 1;
                gTargets[tx_profile][chirp_type][j].rangeBin = hw_rng;
                gTargets[tx_profile][chirp_type][j].velocityBin = hw_vel;
                gTargets[tx_profile][chirp_type][j].realVel = UNVALID_VEL + 1;
        #if TRK_CONF_3D
                gTargets[tx_profile][chirp_type][j].heighAngle = -32.0f;
                gTargets[tx_profile][chirp_type][j].sig_elv = 0.0f;
        #endif
                gTargets[tx_profile][chirp_type][j].doaMag = fl_to_float(hw_sig, 15, 1, false, 5, false);
//                gTargets[tx_profile][chirp_type][j].threshold = fl_to_float(obj_info[i].noi, 15, 1, false, 5, false);
                gTargets[tx_profile][chirp_type][j].threshold = globalNoise;
                gTargets[tx_profile][chirp_type][j].angleNum = 1;
                /* CT End */
                /* FIXME: magic numbers need to be removed */
                track->cdi_pkg.cdi[j].raw_z.sig = fl_to_float(hw_sig, 15, 1, false, 5, false);
                track->cdi_pkg.cdi[j].raw_z.noi = fl_to_float(obj_info[i].noi, 15, 1, false, 5, false);
                if (cfg->track_obj_snr_sel != 0) { // choose RXs FFT peak average power instead of DoA power to calculate SNR
                        uint8_t bpm_idx_min = 0;
//                        uint8_t bpm_idx_max = cfg->nvarray - 1;
                        uint8_t bpm_idx_max = 0; // bpm_idx_max强制为0，只使用4个RX通道数据
                        if (cfg->anti_velamb_en) {
                                bpm_idx_min = 1;
                                bpm_idx_max = cfg->nvarray;
                        }
                        uint32_t old1 = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
                        float fft_power = 0;
                        float fft_power1 = 0;
                        float fft_power2 = 0;
                        uint8_t cnt = 0;
                        for (uint8_t bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                        uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, hw_rng, hw_vel, bpm_index);
                                        complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);
                                        fft_power += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i); // Non-coherent accumulation of 4 channel 2D-FFT power
                                        cnt ++;
                                        gTargets[tx_profile][chirp_type][j].phase[ch_index] = complex_fft.i/complex_fft.r;
                                }
                        }
                        baseband_switch_mem_access(bb_hw, old1);
                        fft_power /= cnt; // Average power
                        track->cdi_pkg.cdi[j].raw_z.sig = fft_power;

                        /* CT Variable set*/
                        gTargets[tx_profile][chirp_type][j].fftMag = fft_power;
                        /* CT end*/

                        // 新增结构体参数用于传输前后一个距离bin的MAG值 -- 20220208 added by zxp 
                        if (hw_rng > 0)
                        {
                                uint8_t bpm_idx_min = 0;
//                                uint8_t bpm_idx_max = cfg->nvarray - 1;
                                uint8_t bpm_idx_max = 0; // bpm_idx_max强制为0，只使用4个RX通道数据
                                if (cfg->anti_velamb_en) {
                                        bpm_idx_min = 1;
                                        bpm_idx_max = cfg->nvarray;
                                }
                                uint32_t old1 = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
                                uint8_t cnt = 0;
                                for (uint8_t bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                        for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, hw_rng - 1, hw_vel, bpm_index);
                                                complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);
                                                fft_power1 += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i); // Non-coherent accumulation of 4 channel 2D-FFT power
                                                cnt ++;
                                        }
                                }
                                baseband_switch_mem_access(bb_hw, old1);
                                fft_power1 /= cnt; // Average power
                        }
                        else
                        {
                                fft_power1 = fft_power;   
                        }
                        if (hw_rng < (cfg->rng_nfft / 2 - 1) )
                        {
                                uint8_t bpm_idx_min = 0;
//                                uint8_t bpm_idx_max = cfg->nvarray - 1;
                                uint8_t bpm_idx_max = 0; // bpm_idx_max强制为0，只使用4个RX通道数据
                                if (cfg->anti_velamb_en)
                                {
                                        bpm_idx_min = 1;
                                        bpm_idx_max = cfg->nvarray;
                                }
                                uint32_t old1 = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
                                uint8_t cnt = 0;
                                for (uint8_t bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                                        for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                                uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw, ch_index, hw_rng + 1, hw_vel, bpm_index);
                                                complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);
                                                fft_power2 += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i); // Non-coherent accumulation of 4 channel 2D-FFT power
                                                cnt ++;
                                        }
                                }
                                baseband_switch_mem_access(bb_hw, old1);
                                fft_power2 /= cnt; // Average power     
                        }
                        else
                        {
                                fft_power2 = fft_power;   
                        }

                        gTargets[tx_profile][chirp_type][j].mag1 = 10 * log10f(sqrtf(fft_power * fft_power + fft_power1 * fft_power1 + fft_power2 * fft_power2));

                        // if (cfg->track_obj_snr_sel != 1) // set noi to 1 to choose fft power instead of SNR for track
                        //         track->cdi_pkg.cdi[j].raw_z.noi = 1;
                }

#if VEL_DEAMB_MF_EN
                if( TRACK_READ_PAIR_ONLY && !( Is_main_obj_match(i) ) ){ //
                        continue;
                } else {
                        q_idx = main_obj_wrap_num(i);
                        if (vel_acc >= sys_params->vel_wrap_max) {
                                q_idx += 1;
                        }
                }

                int32_t hw_noi = obj_info[i].noi;
                OPTION_PRINT("obj_num is %d. \n", obj_num);
                OPTION_PRINT("hw_rng is %d, hw_vel is %d, q_idx is %d, hw_noi is %d, hw_ang is %d\n", \
                                  hw_rng, hw_vel, q_idx, hw_noi, hw_ang);
#elif CUSTOM_VEL_DEAMB_MF
                q_idx = read_wrap_num(i); // Get wrap_num calculated by customized velocity de-ambiguity algorithm
#endif

                if (q_idx > MAX_Q_IDX){
                        q_idx -= MAX_Q_NUM;
                }

#if DDM_EN
                vel_acc = vel_acc + read_pk_pos_shft_tbl(i)*cfg->vel_nfft;
                if (vel_acc >= cfg->nvarray * cfg->vel_nfft/2){
                        vel_acc -= cfg->nvarray * cfg->vel_nfft;
                }
                sys_params->vel_nfft = cfg->vel_nfft;
#else
                /* translate index */
                /* when acc_vel_hw is disabled, non-wrap vel index should be 0 to vel_nfft/2 - 1       */
                /* when acc_vel_hw is enabled , non-wrap vel index should be 0 to vel_nfft/2 - 1 + 0.5 */
                if (vel_acc >= sys_params->vel_wrap_max - 0.5){
                        vel_acc -= sys_params->vel_nfft;
                }
#endif

                if (!(cfg->anti_velamb_en || VEL_DEAMB_MF_EN || CUSTOM_VEL_DEAMB_MF)){
                        //set q = 0 when either anti_vel or de_amb is off
                        q_idx = 0;
                }

                if(cfg->anti_velamb_en)
                {
                        float high_vel_comp_idx = 1.0 * q_idx + vel_acc / (int32_t)sys_params->vel_nfft;
                        // For DDM, in CFAR stage cfg->vel_nfft is divided by DDM_CHRP_PHAS_SEQ_PRD
        #if DDM_EN
                        high_vel_comp_idx /= DDM_CHRP_PHAS_SEQ_PRD;
        #endif
                        vel_acc = vel_acc + 1.0 * q_idx * (int32_t)sys_params->vel_nfft;
                        vel_acc = vel_acc - high_vel_comp_idx * sys_params->vel_comp;
                        rng_acc = rng_acc - high_vel_comp_idx * sys_params->rng_comp;
                        rng_acc = rng_acc < 0 ? 0 : rng_acc;
                }

                radar_param_fft2rv_nowrap(sys_params, rng_acc, vel_acc, &gTargets[tx_profile][chirp_type][j].range, &gTargets[tx_profile][chirp_type][j].velocity);
                /* CT set variable*/
//                gTargets[tx_profile][chirp_type][j].range = track->cdi_pkg.cdi[j].raw_z.rng;

#if (PROFILE_CNT == 4)
                //近距离使用近天线，过滤掉远天线
                if ((tx_profile == 0) && (gTargets[tx_profile][chirp_type][j].range <= 38))
                {
                    continue;
                }
                //远距离使用远程天线
                if ((tx_profile == 1) && (gTargets[tx_profile][chirp_type][j].range > 42))
                {
                    continue;
                }
#endif

                if (is_dml_flag) {
                        if (obj_info[i].doa[0].ang_acc_0 != 0){
                                hw_ang += 512;
                        }

                        /* Type(asin_v):double -> float, asin -> asinf. Processing time optimization */
                        float asin_v = sys_params->dml_sin_az_left + hw_ang * sys_params->dml_sin_az_step;
                        float ang = asinf(asin_v) * RAD2DEG;
                        track->cdi_pkg.cdi[j].raw_z.ang = ang;

                        /* CT Variable set*/
                        gTargets[tx_profile][chirp_type][j].angle = ang;
                        /* CT End*/
                } else {
                        track->cdi_pkg.cdi[j].raw_z.ang = sys_params->bfm_az_left + ang_acc * sys_params->az_delta_deg;
                        /* CT Variable set*/
                        gTargets[tx_profile][chirp_type][j].angle = sys_params->bfm_az_left + ang_acc * sys_params->az_delta_deg;
                        /* CT End*/
                }

                az_angle = ( (track->cdi_pkg.cdi[j].raw_z.ang < sys_params->trk_fov_az_left) || (track->cdi_pkg.cdi[j].raw_z.ang > sys_params->trk_fov_az_right) );
                if (group_num >= 2) {
#if TRK_CONF_3D
                        int32_t hw_ang_elv = obj_info[i].doa[1].ang_idx_0;
                        float ang_acc_elv = fx_to_float(obj_info[i].doa[1].ang_acc_0, 4, 0, true);
                        ang_acc_elv = acc_ang_flag ? (ang_acc_elv + (float)hw_ang_elv) : (float)hw_ang_elv;

                        if (is_dml_flag) {
                                if (obj_info[i].doa[1].ang_acc_0 != 0)
                                        hw_ang_elv += 512;

                                /* Type(asin_v):double -> float, asin -> asinf. Processing time optimization */
                                float asin_v = sys_params->dml_sin_ev_down + hw_ang_elv * sys_params->dml_sin_ev_step;
                                float ang = asinf(asin_v) * RAD2DEG;
                                gTargets[tx_profile][chirp_type][j].heighAngle = ang;
                                track->cdi_pkg.cdi[j].raw_z.ang_elv = ang;
                        } else {
                                gTargets[tx_profile][chirp_type][j].heighAngle = sys_params->bfm_ev_down + ang_acc_elv * sys_params->ev_delta_deg;
                                track->cdi_pkg.cdi[j].raw_z.ang_elv = sys_params->bfm_ev_down + ang_acc_elv * sys_params->ev_delta_deg;
                        }

                        double alpha = track->cdi_pkg.cdi[j].raw_z.ang;
                        double phi = track->cdi_pkg.cdi[j].raw_z.ang_elv;
                        float sin_theta = sinf(alpha * DEG2RAD) / cosf(phi * DEG2RAD);

                        if (sin_theta <= 1 && sin_theta >= -1) {
                                /* Type(sin_theta):double -> float, asin -> asinf. Processing time optimization */
                                gTargets[tx_profile][chirp_type][j].angle = asinf(sin_theta) * RAD2DEG;
                                track->cdi_pkg.cdi[j].raw_z.ang = asinf(sin_theta) * RAD2DEG;
                                az_angle = ( (track->cdi_pkg.cdi[j].raw_z.ang < sys_params->trk_fov_az_left) || (track->cdi_pkg.cdi[j].raw_z.ang > sys_params->trk_fov_az_right) );
                        }

                        uint32_t hw_sig_ev = obj_info[i].doa[1].sig_0;
                        gTargets[tx_profile][chirp_type][j].sig_elv = fl_to_float(hw_sig_ev, 15, 1, false, 5, false);
                        track->cdi_pkg.cdi[j].raw_z.sig_elv = fl_to_float(hw_sig_ev, 15, 1, false, 5, false);
                        elv_angle = ( (track->cdi_pkg.cdi[j].raw_z.ang_elv < sys_params->trk_fov_ev_down) || (track->cdi_pkg.cdi[j].raw_z.ang_elv > sys_params->trk_fov_ev_up) );
#endif
                } else {
#if TRK_CONF_3D
                        track->cdi_pkg.cdi[j].raw_z.ang_elv = 0.0;
                        track->cdi_pkg.cdi[j].raw_z.sig_elv = 0.0;
#endif
                }

                EMBARC_PRINTF("r:%.2f, v:%.2f, a:%.2f\n", gTargets[tx_profile][chirp_type][j].range, 
                    gTargets[tx_profile][chirp_type][j].velocity, gTargets[tx_profile][chirp_type][j].angle);

                if (DUTGetTestState() == DUT_MODE_CLOSE)
                {
                    //根据具体的应用转换角度
                    #if (ANGLE_INVERT_ENABLE==1)
                    gTargets[tx_profile][chirp_type][j].angle = 0.0 - gTargets[tx_profile][chirp_type][j].angle;
                    #else
                    gTargets[tx_profile][chirp_type][j].angle = gTargets[tx_profile][chirp_type][j].angle;
                    #endif
                }
                else
                {
                    //工厂模式使用原始的点值
                    gTargets[tx_profile][chirp_type][j].angle = gTargets[tx_profile][chirp_type][j].angle;
                }

                j++;
                /* pre-filter for obj 0*/
                /* reduce j if some conditions are satisfied */
                //elv_angle = false;   /*comment it out for bug[1145]*/
                angle_invalid = az_angle || elv_angle;
                if (( (hw_sig & 0x1f) > 0x1f ) || (angle_invalid)){    /* ADDED FOR DEBUG PURPOSE */
                        j--;
                        continue;
                }
                /*pre-filter for multi-object*/
                // check multi-obj 0, elevated multi-obj 1
                if (elv_flag) {
                        obj_check(sys_params,
                                  true, obj_info[i].doa[0].ang_idx_0, obj_info[i].doa[0].ang_acc_0, acc_ang_flag, obj_info[i].doa[0].sig_0,
                                  obj_info[i].doa[1].ang_vld_1, obj_info[i].doa[1].ang_idx_1, obj_info[i].doa[1].ang_acc_1, obj_info[i].doa[1].sig_1, elv_flag, &j, gTargets[tx_profile][chirp_type], is_dml_flag);
                }
                // check multi-obj 1 2 3 and other elevated angles
                track_pre_filter(sys_params, doa_info, &j, track_cdi, acc_ang_flag, elv_flag, is_dml_flag);
        }

        /* update other info */
        track->cdi_pkg.raw_number = j;
        track->cdi_pkg.cdi_number = j;

        /* CT Variable set*/
        gDspParam.targetNum[tx_profile][chirp_type] = j;

        for (; j < TRACK_NUM_CDI; ++j) {
                gTargets[tx_profile][chirp_type][j].valid = 0;
                gTargets[tx_profile][chirp_type][j].isUsing = 0;
        }

//        filterObjInvalidSpd(gTargets[tx_profile][chirp_type], gDspParam.targetNum[tx_profile][chirp_type]);

        setFrameFlag(RADAR_INIT_FLAG_ONE_FRAME);
        if (((gProfileCount==4) && (tx_mask_flag & ANT_ALL_CHIRP_MASK) == ANT_ALL_CHIRP_MASK) ||
            ((gProfileCount==2) && (((tx_mask_flag & ANT_TX0_ALL_CHIRP_MASH ) == (ANT_TX0_ALL_CHIRP_MASH)) ||
            (((tx_mask_flag & ANT_TX1_ALL_CHIRP_MASH ) == (ANT_TX1_ALL_CHIRP_MASH))))))
        {
            //满足2次的chirp切换后完成一次解模糊操作，切换顺序tx1 chirp1, tx2 chirp1, 输出数据，tx1 chirp2, tx2 chirp2
            // 这里根据应用配置的chirp特点进行配置,保证最后一个profile进行速度接模糊
            //如果单数时候需要注意，这里需要修改
            if ((track_frame_type == frame_type_profile[1]) || (track_frame_type == frame_type_profile[3]))
            {

                if (track_frame_type == frame_type_profile[1])
                {
                    tx_mask_flag &= (~ANT_TX0_CHIRP1_MASK);
                    tx_mask_flag &= (~ANT_TX1_CHIRP1_MASK);
                }
                if (track_frame_type == frame_type_profile[3])
                {
                    tx_mask_flag &= (~ANT_TX0_CHIRP0_MASK);
                    tx_mask_flag &= (~ANT_TX1_CHIRP0_MASK);
                }

                //只有进行了一次速度解模糊，说明已经可以进行跟踪处理
                setFrameFlag(RADAR_INIT_FLAG_TRACK);
            }
        }
        //解速度模糊
        /* CT End*/

#if TRK_MODE_FUNC == 1
        /* angle calib using NN */
        uint8_t mode_word = baseband_read_reg(NULL, BB_REG_FDB_SYS_BNK_ACT);
        //EMBARC_PRINTF("Mode_word = %d\n\r", mode_word);

        float arg_azi, arg_ele;

        track_trk_pkg_t*    trk_pkg = (track_trk_pkg_t*)track->trk_data;
        trk_pkg->mode_word = mode_word;

        if (0 == mode_word)//srr
        {
                for (i = 0; i < track->cdi_pkg.cdi_number; i++) {
                        arg_azi = track->cdi_pkg.cdi[i].raw_z.ang;
                        //EMBARC_PRINTF("FT = %2.2f\n", tmpAzi);
                        angle_calib_step(&track->cdi_pkg.cdi[i].raw_z.ang, &arg_azi);
                        //EMBARC_PRINTF("FT = %2.2f\n", track->cdi_pkg.cdi[i].raw_z.ang);
                }
        }
        else if (1 == mode_word)//usrr
        {
#if TRK_CONF_3D
                for (i = 0; i < track->cdi_pkg.cdi_number; i++) {
                        arg_azi = track->cdi_pkg.cdi[i].raw_z.ang;
                        arg_ele = track->cdi_pkg.cdi[i].raw_z.ang_elv;
                        //EMBARC_PRINTF("FT = %2.2f\n", tmpAzi);
                        angle_calib_3D_step(arg_azi, arg_ele, &track->cdi_pkg.cdi[i].raw_z.ang, &track->cdi_pkg.cdi[i].raw_z.ang_elv);
                        //EMBARC_PRINTF("FT = %2.2f\n", track->cdi_pkg.cdi[i].raw_z.ang);
                }
#endif
        }

#endif

#if TRK_MODE_FUNC == 1
        //========================ground clutter processing========================//
        if (0 == mode_word)//srr
        {
                clutterProcess2D(track);
        }
        else if (1 == mode_word)//usrr
        {
                clutterProcess3D(track);
        }
#endif

        /* restore memory bank */
        baseband_switch_mem_access(bb_hw, old);

}
//
//
//void track_output_print(track_t *track)
//{
//    int8_t track_output_type = get_track_cfg();
//
//    if (track_output_type == UART_HEX) {
//        track_pkg_uart_hex_print(track);
//    } else if (track_output_type == UART_STRING) {
//        track_pkg_uart_string_print(track);
//    } else if (track_output_type == CAN_POLLING){
//        /* Send the track datas to CAN bus */
//        track_pkg_can_print(track);
//    } else if(track_output_type == CAN_INT){
//        track_pkg_can_int_print(track);
//    } else {
//        EMBARC_PRINTF("track_output_print:type error %d\n", track_output_type);
//    }
//}


/* run tracker for current frame */
void track_run(track_t *track)
{
#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_SLAVE) {
                return;
        }
#endif
        void* sys_params = track->radar_params[baseband_get_cur_frame_type()];
        /* pre run */
//        if (track->trk_pre){
//                track->trk_pre(track->trk_data, sys_params);
//        }
//        track_update_time(track);
//        /* run */
//        if (track->trk_run){
//                track->trk_run(track->trk_data, sys_params);
//        }
//        /* output */
//        track_output_print(track);
//        /* post run */
//        if (track->trk_post){
//                track->trk_post(track->trk_data, sys_params);
//        }
}

static void track_update_time(track_t *track)
{
        if (track->trk_has_run && track->trk_has_run(track->trk_data)) {
                track->f_last_time = track->f_now;
                track->f_now = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
                if (track->trk_set_frame_int){
                        track->trk_set_frame_int(track->trk_data, track->f_now - track->f_last_time);
                }
        } else {
                track->f_now = 1.0f * xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
                track->f_last_time = track->f_now;
                if (track->trk_set_frame_int){
                        track->trk_set_frame_int(track->trk_data, .05);
                }
        }
}
//
///* filter print */
//void track_pkg_uart_string_print(track_t *track)
//{
//        /* variable */
//        uint16_t i;
//        void* sys_params = track->radar_params[baseband_get_cur_frame_type()];
//
//        if (track->trk_update_header){
//                track->trk_update_header(track->trk_data, sys_params);
//        }
//        EMBARC_PRINTF("FT = %2.2f BNK %d\n", track->output_hdr.frame_int, current_frame_type);
//        EMBARC_PRINTF("--- F %d O %d/%d/%d! ---\n", track->output_hdr.frame_id,
//                      track->cdi_pkg.cdi_number,
//                      track->output_hdr.track_output_number,
//                      track->cdi_pkg.raw_number);
//        EMBARC_PRINTF("BK\n");
//        for (i = 0; i < track->cdi_pkg.cdi_number; i++){
//                float tmpS = track->cdi_pkg.cdi[i].raw_z.sig;
//                float tmpN = track->cdi_pkg.cdi[i].raw_z.noi;
//#if (TRK_MODE_FUNC == 1)
//                track->cdi_pkg.cdi[i].raw_z.ang = ((track->cdi_pkg.cdi[i].raw_z.ang - rtU.angle_installation * ANG2RAD) * RAD2ANG);
//#if (TRK_CONF_3D == 1)
//                track->cdi_pkg.cdi[i].raw_z.ang_elv = (track->cdi_pkg.cdi[i].raw_z.ang_elv  * RAD2ANG);
//#endif
//#endif
//#if (TRK_CONF_3D == 0)
//                EMBARC_PRINTF("\t%02d: P %3.2f, R %3.2f, V %3.2f, A %3.2f, E %3.2f\n",
//                              i,
//                              10*log10f(tmpS/tmpN),
//                              track->cdi_pkg.cdi[i].raw_z.rng,
//                              track->cdi_pkg.cdi[i].raw_z.vel,
//                              track->cdi_pkg.cdi[i].raw_z.ang,
//                              (track_float_t)0.0);
//#else
//                EMBARC_PRINTF("\t%02d: P %3.2f, R %3.2f, V %3.2f, A %3.2f, E %3.2f\n",
//                              i,
//                              10*log10f(tmpS/tmpN),
//                              track->cdi_pkg.cdi[i].raw_z.rng,
//                              track->cdi_pkg.cdi[i].raw_z.vel,
//                              track->cdi_pkg.cdi[i].raw_z.ang,
//                              track->cdi_pkg.cdi[i].raw_z.ang_elv);
//#endif
//        }
//        EMBARC_PRINTF("AK\n");
//        for (i = 0; i < TRACK_NUM_TRK; i++) {
//                if (track->trk_update_obj_info){
//                        track->trk_update_obj_info(track->trk_data, sys_params, i);
//                }
//                if (track->output_obj.output) {
//#if (TRK_CONF_3D == 0)
//                        EMBARC_PRINTF("\t%02d: P %3.2f, R %3.2f, V %3.2f, A %3.2f, S %d, E %3.2f, L %d - F %d\n",
//                                i,
//                                track->output_obj.SNR,
//                                track->output_obj.rng,
//                                track->output_obj.vel,
//                                track->output_obj.ang,
//                                track->output_obj.track_level,
//                                (track_float_t)0.0,
//                                (uint32_t)0,
//#if (TRK_MODE_FUNC == 1)
//                                (uint32_t)(rtDW.TrackLib[i].classifyID << 2));  /* zeros for E/L/F */
//#else
//                                (uint32_t)0);  /* zeros for E/L/F */
//#endif
//#else
//                        EMBARC_PRINTF("\t%02d: P %3.2f, R %3.2f, V %3.2f, A %3.2f, S %d, E %3.2f, L %d - F %d\n",
//                                      i,
//                                      track->output_obj.SNR,
//                                      track->output_obj.rng,
//                                      track->output_obj.vel,
//                                      track->output_obj.ang,
//                                      track->output_obj.track_level,
//                                      track->output_obj.ang_elv,
//                                      (uint32_t)0,
//                                      (uint32_t)0);     /* zeros for L/F */
//#endif
//
//                }
//        }
//        EMBARC_PRINTF("#\n\n");
//}


static void init_track_data(uint16_t number)
{
        /* variable */
        uint16_t i = 0;

        trk_hex.data = (uint32_t *) pvPortMalloc(number * sizeof(uint32_t));
        if (trk_hex.data != NULL) {
                for (i = 0; i < number; i++){
                        trk_hex.data[i] = 0x0;
                }
        }
}


uint32_t transform_data(float data)
{
        uint32_t value = 0;

        value = (uint32_t)(0x4000 + ((data * 100) + ROUNDF));

        return value;
}


static void init_data_zero(uint32_t *track_data, uint32_t start_index, uint32_t end_index)
{
        uint16_t i = 0;

        for(i = start_index; i < end_index; i++){
                track_data[i] = 0x0;
        }
}

void tx_end_handle(void) {
        /* unlock the entire frame space */
        trk_hex.lock = false;
}
//
//static void transfer_data(track_t *track, data_block_t data, uint16_t index, uint32_t end)
//{
//        /* variable */
//        uint16_t array_num = 0;
//
//        switch (data) {
//        case HEAD:
//                /* Checksum of Head */
//                trk_hex.data[5] = ((trk_hex.data[2] + trk_hex.data[3]) & LOW32BITMASk);
//                /* End of Head */
//                trk_hex.data[6] = end;
//                array_num = 7;
//                break;
//        case BK:
//                if (track->cdi_pkg.cdi_number == 0) {
//                        /* End of BK(Before Kalman) */
//                        trk_hex.data[1] = end;    /* In this case, Checksum = 0x0 */
//                        array_num = 2;
//                } else {
//                        /* Checksum of BK */
//                        trk_hex.data[index + 6] = ((trk_hex.data[2] + trk_hex.data[index + 2]) & LOW32BITMASk) ;
//                        /* End of BK(Before Kalman) */
//                        trk_hex.data[index + 7] = end;
//                        array_num = index + 8;
//                }
//                break;
//        case AK:
//                if (track->output_hdr.track_output_number == 0) {
//                        trk_hex.data[1] = end;    /* In this case, Checksum = 0x0 */
//                        array_num = 2;
//                } else {
//                        /* Checksum of AK */
//                        trk_hex.data[index + 6] = ((trk_hex.data[2] + trk_hex.data[index + 2]) & LOW32BITMASk);
//                        /* End of AK(After Kalman) */
//                        trk_hex.data[index + 7] = end;
//                        array_num = index + 8;
//                }
//                break;
//        default:
//                // add comment in default £¬rule 16.4
//                break;
//        }
//
//        /* Data Transfer to Host through UART */
//        portENTER_CRITICAL();
//        bprintf(trk_hex.data, (array_num * sizeof(uint32_t)), tx_end_handle);
//
//        /* lock the trk_hex.data buffer */
//        trk_hex.lock = true;
//        portEXIT_CRITICAL();
//}

static bool query_buf(void)
{
        do {
                if (trk_hex.lock) {
                        /* wait for the previous frame transmission end*/
                        chip_hw_udelay(20);
                } else {
                        return true;
                }
        } while (1);
}
//
///* track packages data print */
//void track_pkg_uart_hex_print(track_t *track)
//{
//        /* variable */
//        uint16_t i = 0, j = 0, index = 0;
//        void* sys_params = track->radar_params[baseband_get_cur_frame_type()];
//
//        if (track->trk_update_header){
//                track->trk_update_header(track->trk_data, sys_params);
//        }
//
//        /* The tracking information can only be filled
//         * in when trk_hex.data buffer status is unlocked.
//         * */
//        if (query_buf()) {
//                /* Head Data - 7 WORD Length : From trk_hex.data[0] to trk_hex.data[6] */
//                /* Initialize trk_hex.data from 0 to MAX_ARRAY_NUM */
//                init_data_zero(trk_hex.data, 0, 6);
//                /* 0xFFEEFFDC is SOH (Start of Head) */
//                trk_hex.data[0] = 0xFFEEFFDC;
//                trk_hex.data[1] = ((uint32_t)((track->output_hdr.frame_int * 100) + ROUNDF) & LOW10BITMASk);
//                trk_hex.data[1] |= ((track->output_hdr.frame_type & 0x07) << 10);
//                trk_hex.data[2] = track->output_hdr.frame_id;
//                trk_hex.data[3] = ((track->cdi_pkg.cdi_number & LOW10BITMASk) |
//                                ((track->output_hdr.track_output_number & LOW10BITMASk) << 10) |
//                                ((track->cdi_pkg.raw_number & LOW10BITMASk) << 20));
//                trk_hex.data[4] = 0x0;
//                /* 0xFFEEFFD3 is EOH(End of Head) */
//                transfer_data(track, HEAD, 0, 0xFFEEFFD3);
//        }
//
//        if (query_buf()) {
//                /* BK Data */
//                /* Initialize trk_hex.data from 0 to MAX_ARRAY_NUM */
//                init_data_zero(trk_hex.data, 0, MAX_ARRAY_NUM);
//                /* 0xFFDDFECB is SOB (Start of BK) */
//                trk_hex.data[0] = 0xFFDDFECB;
//                for (i = 0; i < track->cdi_pkg.cdi_number; i++) {
//                        index = 5 * i;
//                        float tmpS = track->cdi_pkg.cdi[i].raw_z.sig;
//                        float tmpN = track->cdi_pkg.cdi[i].raw_z.noi;
//                        float SNR = 10 * log10f(tmpS / tmpN);
//#if (TRK_MODE_FUNC == 1)
//                        track->cdi_pkg.cdi[i].raw_z.ang = ((track->cdi_pkg.cdi[i].raw_z.ang - rtU.angle_installation * ANG2RAD) * RAD2ANG);
//#if (TRK_CONF_3D == 1)
//                        track->cdi_pkg.cdi[i].raw_z.ang_elv = (track->cdi_pkg.cdi[i].raw_z.ang_elv  * RAD2ANG);
//#endif
//#endif
//                        trk_hex.data[(index + 1)] = (i & LOW10BITMASk) | ((transform_data(SNR) & LOW15BITMASk) << 10);
//                        trk_hex.data[(index + 2)] = (uint32_t)((track->cdi_pkg.cdi[i].raw_z.rng * 100) + ROUNDF);
//                        trk_hex.data[(index + 3)] = ((transform_data(track->cdi_pkg.cdi[i].raw_z.vel) & LOW15BITMASk) |
//                                                  ((transform_data(track->cdi_pkg.cdi[i].raw_z.ang) & LOW15BITMASk) << 15));
//                #if (TRK_CONF_3D == 0)
//                        trk_hex.data[(index + 4)] = 0x4000;
//                #else
//                        trk_hex.data[(index + 4)] = transform_data(track->cdi_pkg.cdi[i].raw_z.ang_elv) & LOW15BITMASk;
//                #endif
//                        trk_hex.data[(index + 5)] = 0x0;
//                }
//                /* 0xFFDDFEC4 is EOB(End of BK) */
//                transfer_data(track, BK, index, 0xFFDDFEC4);
//        }
//
//        if (query_buf()) {
//                /* AK Data */
//                /* Initialize trk_hex.data from 0 to MAX_ARRAY_NUM */
//                init_data_zero(trk_hex.data, 0, MAX_ARRAY_NUM);
//                /* 0xFFCCFDBA is SOA (Start of AK) */
//                trk_hex.data[0] = 0xFFCCFDBA;
//                for (i = 0; i < TRACK_NUM_TRK; i++) {
//                        if (track->trk_update_obj_info){
//                                track->trk_update_obj_info(track->trk_data, sys_params, i);
//                        }
//                        if (track->output_obj.output) {
//                                index = 5 * j++;
//                                trk_hex.data[(index + 1)] = (i & LOW10BITMASk) | ((transform_data(track->output_obj.SNR) & LOW15BITMASk) << 10);
//                                trk_hex.data[(index + 2)] = (uint32_t)((track->output_obj.rng  * 100) + ROUNDF);
//                                trk_hex.data[(index + 3)] = ((transform_data(track->output_obj.vel) & LOW15BITMASk)
//                                        | ((transform_data(track->output_obj.ang) & LOW15BITMASk) << 15)
//                                        | ((track->output_obj.track_level & LOW2BITMASk) << 30));
//#if (TRK_CONF_3D == 0)
//                                //trk_hex.data[(index + 4)] = 0x4000;
//#if (TRK_MODE_FUNC == 1)
//                                trk_hex.data[(index + 4)] = 0x4000 | (rtDW.TrackLib[i].classifyID << 15);
//                                //trk_hex.data[(index + 4)] = 0x4000 | (0xFF << 15);
//#else
//                                trk_hex.data[(index + 4)] = 0x4000;
//#endif
//#else
//#if (TRK_MODE_FUNC == 1)
//                                trk_hex.data[(index + 4)] = (transform_data(track->output_obj.ang_elv) & LOW15BITMASk) | (rtDW.TrackLib[i].classifyID << 15);
//#else
//                                trk_hex.data[(index + 4)] = (transform_data(track->output_obj.ang_elv) & LOW15BITMASk);
//#endif
//#endif
//                                trk_hex.data[(index + 5)] = 0x0;
//                        }
//                }
//                /* 0xFFCCFDB5 is EOA(End of AK) */
//                transfer_data(track, AK, index, 0xFFCCFDB5);
//        }
//
//        return;
//}

#if 0
/* TODO, following 3 functions(switch timer period) can be removed, just reserved here for future possible needs */
/* They are used for bug fixing in ALPS_B, but they're unuseful for now in ALPS_MP */

static uint8_t old_track_fps;
void track_timer_period_save()
{
        old_track_fps = 1000/ (xTimerGetPeriod(xTimerTrack));
}

void track_timer_period_change(uint8_t new_period)
{
         if( xTimerIsTimerActive(xTimerTrack) != pdFALSE ) /* check whether timer is active or not */
                 xTimerStop(xTimerTrack, 0); /* xTimer is active, stop it. */
         xTimerChangePeriod( xTimerTrack, 1000/new_period, 0); /* 0 --> change immediately*/
}

void track_timer_period_restore()
{
        track_timer_period_change(old_track_fps);
}
#endif
