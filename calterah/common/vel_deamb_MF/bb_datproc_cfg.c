#include "vel_deamb_MF.h"

#if VEL_DEAMB_MF_EN

void bb_dpc_pre(baseband_t * bb)
{
        frame_type_params_update(bb);
}

void bb_dpc_post_irq(baseband_t * bb)
{
        obj_pair(bb);
}

void bb_dpc_config(baseband_data_proc_t * bb_dpc){
        uint8_t frame_ind;
        for (frame_ind = 0; frame_ind < 2 && frame_ind < DPC_SIZE ; frame_ind ++) {
                bb_dpc[frame_ind].pre = bb_dpc_pre;
                bb_dpc[frame_ind].post = NULL;
                bb_dpc[frame_ind].post_irq = bb_dpc_post_irq;
                bb_dpc[frame_ind].fi_recfg = true;
                bb_dpc[frame_ind].stream_on = true;
                bb_dpc[frame_ind].radio_en = true;
                bb_dpc[frame_ind].tx_en = true;
                sensor_config_t* cfg = sensor_config_get_config(frame_ind);
                bb_dpc[frame_ind].sys_enable =
                        SYS_ENA(AGC   , cfg->agc_mode)
                        |SYS_ENA(SAM   , true         )
                        |SYS_ENA(FFT_2D, true         )
                        |SYS_ENA(CFR   , true         );
                bb_dpc[frame_ind].cas_sync_en = true;
                bb_dpc[frame_ind].sys_irq_en = BB_IRQ_ENABLE_ALL;
                bb_dpc[frame_ind].track_en = false;
                bb_dpc[frame_ind].end = false;
        }
        bb_dpc[frame_ind].pre = NULL;
        bb_dpc[frame_ind].post = NULL;
        bb_dpc[frame_ind].post_irq = NULL;
        bb_dpc[frame_ind].fi_recfg = true;
        bb_dpc[frame_ind].stream_on = false;
        bb_dpc[frame_ind].radio_en = false;
        bb_dpc[frame_ind].tx_en = false;
        bb_dpc[frame_ind].sys_enable = SYS_ENA(BFM   , true         );
        bb_dpc[frame_ind].cas_sync_en = false;
        bb_dpc[frame_ind].sys_irq_en = BB_IRQ_ENABLE_ALL;
        bb_dpc[frame_ind].track_en = true;
        bb_dpc[frame_ind].end = true;
}

#endif

#if CUSTOM_VEL_DEAMB_MF
#define FRAME_TYPE_NUM 2 // Typical 2 frames velocity de-ambiguity
/*
 * Function before baseband begins to run
 * */
void bb_dpc_pre(baseband_t * bb)
{
        uint8_t current_frame_type = baseband_read_reg(NULL, BB_REG_FDB_SYS_BNK_ACT);
        if (current_frame_type == 0) {
                ; // Actions before the first frame baseband runs
        } else if(current_frame_type == 1) {
                ; // Actions before the second frame baseband runs
        }
}

/*
 * Function after baseband runs to end
 * */
void bb_dpc_post_irq(baseband_t * bb)
{
        uint8_t current_frame_type = baseband_read_reg(NULL, BB_REG_FDB_SYS_BNK_ACT);
        if (current_frame_type == 0) {
                ; // Actions after the first frame baseband runs to end
        }else if(current_frame_type == 1) {
                ;// Add customer implemented velocity de-ambiguity algorithms here.
                write_wrap_num(bb); // write Doppler domain wrap number into MEM_RLT
        }
}

/*
 * Configure baseband working flow.
 * For the typical 2 frames interleaving velocity de-ambiguity, the baseband needs run 3 times:
 * For the first and second time, baseband runs SAM, FFT2D and CFAR with different chirp periods configurations.
 * For the last time, baseband runs DOA only, based on the second time FFT2D data.
 * */
void bb_dpc_config(baseband_data_proc_t * bb_dpc){
        uint8_t frame_ind = 0;
        for (frame_ind = 0; frame_ind < FRAME_TYPE_NUM && frame_ind < DPC_SIZE ; frame_ind ++) {
                bb_dpc[frame_ind].pre = bb_dpc_pre;
                bb_dpc[frame_ind].post = NULL;
                bb_dpc[frame_ind].post_irq = bb_dpc_post_irq;
                bb_dpc[frame_ind].fi_recfg = true;
                bb_dpc[frame_ind].stream_on = true;
                bb_dpc[frame_ind].radio_en = true;
                bb_dpc[frame_ind].tx_en = true;
                sensor_config_t* cfg = sensor_config_get_config(frame_ind);
                bb_dpc[frame_ind].sys_enable =
                        SYS_ENA(AGC   , cfg->agc_mode)
                        |SYS_ENA(SAM   , true         )
                        |SYS_ENA(FFT_2D, true         )
                        |SYS_ENA(CFR   , true         );
                bb_dpc[frame_ind].cas_sync_en = true;
                bb_dpc[frame_ind].sys_irq_en = BB_IRQ_ENABLE_ALL;
                bb_dpc[frame_ind].track_en = false;
                bb_dpc[frame_ind].end = false;
        }
        bb_dpc[frame_ind].pre = NULL;
        bb_dpc[frame_ind].post = NULL;
        bb_dpc[frame_ind].post_irq = NULL;
        bb_dpc[frame_ind].fi_recfg = false;
        bb_dpc[frame_ind].stream_on = false;
        bb_dpc[frame_ind].radio_en = false;
        bb_dpc[frame_ind].tx_en = false;
        bb_dpc[frame_ind].sys_enable = SYS_ENA(BFM   , true         );
        bb_dpc[frame_ind].cas_sync_en = false;
        bb_dpc[frame_ind].sys_irq_en = BB_IRQ_ENABLE_ALL;
        bb_dpc[frame_ind].track_en = true;
        bb_dpc[frame_ind].end = true;
}

#endif

