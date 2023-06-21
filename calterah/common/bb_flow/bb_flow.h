#ifndef BB_FLOW_H
#define BB_FLOW_H

#include "sensor_config.h"
#include "baseband_task.h"
#include "baseband.h"
#include "alps_hardware.h"
#include "embARC_debug.h"
#include <stdlib.h>
#include "baseband_alps_FM_reg.h"
#include "baseband_hw.h"
#include "radio_reg.h"
#include "radio_ctrl.h"
#include <math.h>
#include "track.h"
#include "baseband_dpc.h"
#include "calterah_complex.h"
#include "calterah_math_funcs.h"


#define DDM_EN    false     // Enable newly added application


#define DDM_CHRP_PHAS_SEQ_PRD             4

void app_dpc_pre(baseband_t * bb);
void app_dpc_post_irq(baseband_t * bb);

void app_dpc_post_fft2d_irq(baseband_t * bb);
void app_dpc_pre_cfar(baseband_t * bb);
void app_dpc_post_cfar_irq(baseband_t * bb);
void app_dpc_post_doa_irq(baseband_t * bb);

void app_dpc_config(baseband_data_proc_t * bb_dpc);

void fmcw_radio_DDM_cmd_cfg(fmcw_radio_t *radio);
void init_pk_pos_shft_tbl();
uint8_t read_pk_pos_shft_tbl(int obj_ind);
void search_ddm_pk_pos(baseband_t *bb);

void ddm_tx_pos2tx_group(sensor_config_t* cfg);


void switch_to_ddm_config(baseband_hw_t *bb_hw);
void recover_from_ddm_config(baseband_hw_t *bb_hw);


#endif
