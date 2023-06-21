#include "bb_flow.h"
#include "option_print.h"
#include "calterah_steering_vector.h"

#if DDM_EN  // bb_dpc configurations for DDM MIMO

static uint8_t init_nvarray[MAX_FRAME_TYPE];
static uint16_t init_vel_nfft[MAX_FRAME_TYPE];
static uint32_t init_tx_groups[MAX_FRAME_TYPE][MAX_NUM_TX];

// Following flags ensure that init_XXX[MAX_FRAME_TYPE] are initialized only once
static bool init_nvarray_flag[MAX_FRAME_TYPE] = {1, 1, 1, 1};
static bool init_vel_nfft_flag[MAX_FRAME_TYPE] = {1, 1, 1, 1};
static bool init_tx_groups_flag[MAX_FRAME_TYPE] = {1, 1, 1, 1};

static void write_init_nvarray(baseband_hw_t *bb_hw, uint8_t cfg_nvarray)
{
        uint8_t frame_type_id = bb_hw->frame_type_id;
        if (init_nvarray_flag[frame_type_id]) {
                init_nvarray[frame_type_id] = cfg_nvarray;
                init_nvarray_flag[frame_type_id] = 0;
        } else {
                return;
        }
}
static void write_init_vel_nfft(baseband_hw_t *bb_hw, uint16_t cfg_vel_nfft) {
        uint8_t frame_type_id = bb_hw->frame_type_id;
        if (init_vel_nfft_flag[frame_type_id]) {
                init_vel_nfft[frame_type_id] = cfg_vel_nfft;
                init_vel_nfft_flag[frame_type_id] = 0;
        } else {
                return;
        }
}
static void write_init_tx_groups(baseband_hw_t *bb_hw, uint32_t* cfg_tx_groups) {
       uint8_t frame_type_id = bb_hw->frame_type_id;
       if (init_tx_groups_flag[frame_type_id]) {
                for (uint8_t t = 0; t < MAX_NUM_TX; t++) {
                        init_tx_groups[frame_type_id][t] = cfg_tx_groups[t];
                }
                init_tx_groups_flag[frame_type_id] = 0;
        } else {
                return;
        }
}

static uint8_t read_init_nvarray(baseband_hw_t *bb_hw)
{
        uint8_t frame_type_id = bb_hw->frame_type_id;
        return init_nvarray[frame_type_id];
}
static uint16_t read_init_vel_nfft(baseband_hw_t *bb_hw) {
        uint8_t frame_type_id = bb_hw->frame_type_id;
        return init_vel_nfft[frame_type_id];
}
static uint32_t* read_init_tx_groups(baseband_hw_t *bb_hw) {
        uint8_t frame_type_id = bb_hw->frame_type_id;
        return init_tx_groups[frame_type_id];
}

void switch_to_ddm_config(baseband_hw_t *bb_hw)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(bb_hw, baseband_t, bb_hw)->cfg;
        write_init_nvarray(bb_hw, cfg->nvarray);
        write_init_vel_nfft(bb_hw, cfg->vel_nfft);
        write_init_tx_groups(bb_hw, cfg->tx_groups);
        cfg->nvarray = DDM_CHRP_PHAS_SEQ_PRD;
        cfg->vel_nfft = cfg->vel_nfft/DDM_CHRP_PHAS_SEQ_PRD;
        ddm_tx_pos2tx_group(cfg);
        for (uint8_t t = 0; t < MAX_NUM_TX; t++) {
                EMBARC_PRINTF("cfg->tx_groups [%d] = %d \n", t, cfg->tx_groups [t]);
        }
        reset_ant_pos_print_flag();
}
void recover_from_ddm_config(baseband_hw_t *bb_hw)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(bb_hw, baseband_t, bb_hw)->cfg;
        cfg->nvarray = read_init_nvarray(bb_hw);
        cfg->vel_nfft = read_init_vel_nfft(bb_hw);
        uint32_t* tmp = read_init_tx_groups(bb_hw);
        for (uint8_t t = 0; t < MAX_NUM_TX; t++) {
                cfg->tx_groups [t] = tmp[t];
                EMBARC_PRINTF("cfg->tx_groups [%d] = %d \n", t, cfg->tx_groups [t]);
        }
}

void app_dpc_pre(baseband_t * bb)
{
        OPTION_PRINT("\n app_dpc_pre \n");
        sensor_config_t* cfg = (sensor_config_t*)(bb->cfg);
        baseband_hw_t* bb_hw = &bb->bb_hw;
        uint8_t frame_type_id = bb_hw->frame_type_id;
        cfg->nvarray = init_nvarray[frame_type_id];
        cfg->vel_nfft = init_vel_nfft[frame_type_id];

        BB_WRITE_REG(bb_hw, SYS_SIZE_BPM    , cfg->nvarray - 1);
        BB_WRITE_REG(bb_hw, SYS_SIZE_VEL_FFT, cfg->vel_nfft - 1);

        OPTION_PRINT("\n cfg->nvarray = %d, cfg->vel_nfft = %d \n", cfg->nvarray, cfg->vel_nfft);
}

void app_dpc_post_fft2d_irq(baseband_t * bb)
{
        OPTION_PRINT("\n app_dpc_post_fft2d_irq \n");
}

void app_dpc_pre_cfar(baseband_t * bb)
{
        OPTION_PRINT("\n app_dpc_pre_cfar \n");
        sensor_config_t* cfg = (sensor_config_t*)(bb->cfg);
        baseband_hw_t* bb_hw = &bb->bb_hw;
        uint8_t frame_type_id = bb_hw->frame_type_id;
        cfg->nvarray = DDM_CHRP_PHAS_SEQ_PRD;
        cfg->vel_nfft = init_vel_nfft[frame_type_id]/DDM_CHRP_PHAS_SEQ_PRD;
        BB_WRITE_REG(bb_hw, SYS_SIZE_BPM    , cfg->nvarray - 1);
        BB_WRITE_REG(bb_hw, SYS_SIZE_VEL_FFT, cfg->vel_nfft - 1);

        OPTION_PRINT("\n cfg->nvarray = %d, cfg->vel_nfft = %d \n", cfg->nvarray, cfg->vel_nfft);
}

void app_dpc_post_cfar_irq(baseband_t * bb)
{
        OPTION_PRINT("\n app_dpc_post_cfar_irq \n");

        search_ddm_pk_pos(bb);
}

void app_dpc_post_doa_irq(baseband_t * bb)
{
        OPTION_PRINT("\n app_dpc_post_doa_irq \n");
}

void app_dpc_config(baseband_data_proc_t * bb_dpc){

        bb_dpc[0].pre = app_dpc_pre;
        bb_dpc[0].post = NULL;
        bb_dpc[0].post_irq = app_dpc_post_fft2d_irq;
        bb_dpc[0].fi_recfg = true;   // switch to bank 0 configurations
        bb_dpc[0].stream_on = true;
        bb_dpc[0].radio_en = true;
        bb_dpc[0].tx_en = true;
        sensor_config_t* cfg = sensor_config_get_config(0);
        bb_dpc[0].sys_enable =
                        SYS_ENA(AGC   , cfg->agc_mode)
                        |SYS_ENA(SAM   , true         )     // BB runs Sampling and 2D-FFT
                        |SYS_ENA(FFT_2D, true         );
        bb_dpc[0].cas_sync_en = false;
        bb_dpc[0].sys_irq_en = BB_IRQ_ENABLE_ALL;
        bb_dpc[0].track_en = true;
        bb_dpc[0].end = false;

        bb_dpc[1].pre = app_dpc_pre_cfar;
        bb_dpc[1].post = NULL;
        bb_dpc[1].post_irq = app_dpc_post_cfar_irq;
        bb_dpc[1].fi_recfg = false;
        bb_dpc[1].stream_on = false;
        bb_dpc[1].radio_en = false;
        bb_dpc[1].tx_en = false;
        bb_dpc[1].sys_enable = SYS_ENA(CFR   , true         );  // BB runs CFAR only
        bb_dpc[1].cas_sync_en = false;
        bb_dpc[1].sys_irq_en = BB_IRQ_ENABLE_ALL;
        bb_dpc[1].track_en = false;
        bb_dpc[1].end = false;

        bb_dpc[2].pre = NULL;
        bb_dpc[2].post = NULL;
        bb_dpc[2].post_irq = NULL; // last post_irq function is located in baseband_task.c
        bb_dpc[2].fi_recfg = false; // use frame type 1 configurations
        bb_dpc[2].stream_on = false;
        bb_dpc[2].radio_en = false;
        bb_dpc[2].tx_en = false;
        bb_dpc[2].sys_enable = SYS_ENA(BFM   , true         );   // BB runs DOA only
        bb_dpc[2].cas_sync_en = false;
        bb_dpc[2].sys_irq_en = BB_IRQ_ENABLE_ALL;
        bb_dpc[2].track_en = false;
        bb_dpc[2].end = true;
}

#endif
