
#include "bb_flow.h"
#include "option_print.h"

void NO_PRINTF(const char *format,...){}

#if DDM_EN

#define FFT_AMP_SCAL 1000
#define FFT_POW_SCAL 1000000

/* Peak position orders of different TXs in Doppler domain */
//static uint8_t ddm_tx_vel_pos[MAX_NUM_TX] = {1, 0, 0, 2};  // For Venus, change tx_group accordingly
//static uint8_t ddm_tx_vel_pos[MAX_NUM_TX] = {0, 0, 1, 2};   // For Lyra, change tx_group accordingly
static uint8_t ddm_tx_vel_pos[MAX_NUM_TX] = {1, 2, 3, 4};   // Enable all TXs' phase shifters
static uint8_t tx_on_num[MAX_FRAME_TYPE] = {0, 0, 0, 0};

void ddm_tx_pos2tx_group(sensor_config_t* cfg) {
        uint8_t tx_order_idx = DDM_CHRP_PHAS_SEQ_PRD;
        for(uint8_t t = 0; t < MAX_NUM_TX; t++) {   // e.g. convert 'ddm_tx_vel_pos={0,0,1,2}' to 'tx_groups={4096,256,1,16}'
            if (ddm_tx_vel_pos[t] == 0) {
                    cfg->tx_groups[t] = 1u<<((tx_order_idx-1)*4);
                    tx_order_idx--;
            } else {
                    cfg->tx_groups[t] = 1u<<((ddm_tx_vel_pos[t]-1)*4);
            }
        }
}

/* Chirp phase shift sequence to generate various shift in Doppler domain */
#if DDM_CHRP_PHAS_SEQ_PRD == 2

static uint32_t TXs_phase_seq[MAX_NUM_TX - 1][DDM_CHRP_PHAS_SEQ_PRD] = {TX_PHASE_225, TX_PHASE_45 }; // PI shift in Doppler domain

#elif DDM_CHRP_PHAS_SEQ_PRD == 4

static uint32_t TXs_phase_seq[MAX_NUM_TX - 1][DDM_CHRP_PHAS_SEQ_PRD] = {TX_PHASE_135, TX_PHASE_225, TX_PHASE_315, TX_PHASE_45,  // PI/2   shift in Doppler domain
                                                                        TX_PHASE_225, TX_PHASE_45,  TX_PHASE_225, TX_PHASE_45,  // PI     shift in Doppler domain
                                                                        TX_PHASE_315, TX_PHASE_225, TX_PHASE_135, TX_PHASE_45}; // 3*PI/2 shift in Doppler domain
#elif DDM_CHRP_PHAS_SEQ_PRD == 8

static uint32_t TXs_phase_seq[MAX_NUM_TX - 1][DDM_CHRP_PHAS_SEQ_PRD] = {TX_PHASE_90,  TX_PHASE_135,  TX_PHASE_180, TX_PHASE_225,
                                                                         TX_PHASE_270, TX_PHASE_315,  TX_PHASE_0,   TX_PHASE_45}; // PI/4 shift in Doppler domain
#endif


static uint8_t ddm_pk_pos_shft_tbl[TRACK_NUM_CDI];  // Recording the distance between CFAR detected peak and the true peak of TX with no chirp phase shift
                                                    // The distance is recorded as times of cfg->vel_nfft in CFAR stage, not in Sample and 2D-FFT stage

void init_pk_pos_shft_tbl() {
        for (int obj_ind = 0; obj_ind < TRACK_NUM_CDI; obj_ind ++) {
                ddm_pk_pos_shft_tbl[obj_ind] = 0;
        }
}

uint8_t read_pk_pos_shft_tbl(int obj_ind) {
        return ddm_pk_pos_shft_tbl[obj_ind];
}

/*
 * Get MIMO RXs FFT vector from 2M FFT memory
 * cfg->nvarray should equal to DDM_CHRP_PHAS_SEQ_PRD
 * */
static void get_rx_fft_vec(baseband_t * bb, uint32_t * fft_vec, int rng_index, int vel_index)
{
        baseband_hw_t *bb_hw = &(bb->bb_hw);
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(bb_hw, baseband_t, bb_hw)->cfg;

        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
        uint8_t bpm_idx_min = 0;
        uint8_t bpm_idx_max = cfg->nvarray - 1;
        if (cfg->anti_velamb_en) {
                bpm_idx_min = 1;
                bpm_idx_max = cfg->nvarray;
        }

        OPTION_PRINT("rng_index = %d \n", rng_index);
        for (uint8_t bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                for(uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                        fft_vec[(bpm_index - bpm_idx_min)*MAX_NUM_RX + ch_index] = baseband_hw_get_fft_mem(bb_hw, ch_index, rng_index, vel_index, bpm_index);
                        OPTION_PRINT("bpm_index = %d, ch_index = %d, ", bpm_index, ch_index);
                        OPTION_PRINT("fft_vec[%d] = %u \n", (bpm_index - bpm_idx_min)*MAX_NUM_RX + ch_index, fft_vec[(bpm_index - bpm_idx_min)*MAX_NUM_RX + ch_index]);
#if PRINT_EN
                        complex_t complex_fft = cfl_to_complex(fft_vec[(bpm_index - bpm_idx_min)*MAX_NUM_RX + ch_index], 14, 14, true, 4, false);
                        float angle = (atan2f(complex_fft.i, complex_fft.r) * 180.0 / 3.14);
                        float power = (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i) / 50;
                        OPTION_PRINT("ang = %7.2f, pow = %7.2f \n", angle, power);
#endif
                }
        }

        baseband_switch_mem_access(bb_hw, old);
}

/*
 * read fft result in memory, all the input index should be based on 0
 * TODO move this function definition to baseband_hw.c
 * */
static void baseband_hw_write_fft_mem(baseband_hw_t *bb_hw, int ant_index, int rng_index, int vel_index, int bpm_index, uint32_t value)
{
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(bb_hw, baseband_t, bb_hw)->cfg;
        uint32_t addr_sel;

        /* TVAR, [virTual][Velocity][Antenna][Range] */
        addr_sel = bpm_index * (cfg->vel_nfft) * MAX_NUM_RX * (cfg->rng_nfft / 2)
                             + vel_index       * MAX_NUM_RX * (cfg->rng_nfft / 2)
                                               + ant_index  * (cfg->rng_nfft / 2)
                                                            + rng_index;
        baseband_write_mem_table(bb_hw, addr_sel, value);
}

/*
 * Rewrite MIMO RXs FFT vector to 2M FFT memory according to true peak position shift
 * This is a pre-process before TDM-MIMO DOA for DDM-MIMO
 * cfg->nvarray should equal to DDM_CHRP_PHAS_SEQ_PRD
 * */
static void arrange_rx_fft_vec(baseband_t * bb, uint32_t * fft_vec, int rng_index, int vel_index, uint8_t va_shift)
{
        baseband_hw_t *bb_hw = &(bb->bb_hw);
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(bb_hw, baseband_t, bb_hw)->cfg;

        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
        uint8_t bpm_idx_min = 0;
        uint8_t bpm_idx_max = cfg->nvarray - 1;
        if (cfg->anti_velamb_en) {
                bpm_idx_min = 1;
                bpm_idx_max = cfg->nvarray;
        }
        for (uint8_t bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                for(uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                        uint32_t tmp_val = fft_vec[(((bpm_index - bpm_idx_min) + va_shift) % DDM_CHRP_PHAS_SEQ_PRD)*MAX_NUM_RX + ch_index];
                        baseband_hw_write_fft_mem(bb_hw, ch_index, rng_index, vel_index, bpm_index, tmp_val);
                }
        }
        baseband_switch_mem_access(bb_hw, old);
}


/*
 * Search for the true velocity index
 * */
void search_ddm_pk_pos(baseband_t *bb)
{
        baseband_hw_t *bb_hw = &(bb->bb_hw);
        sensor_config_t* cfg = (sensor_config_t*)CONTAINER_OF(bb_hw, baseband_t, bb_hw)->cfg;

        uint8_t current_frame_type = baseband_read_reg(NULL, BB_REG_FDB_SYS_BNK_ACT); // Get current frame type (A or B)
        int obj_num = BB_READ_REG(bb_hw, CFR_NUMB_OBJ);
        if( obj_num > TRACK_NUM_CDI )
        obj_num = TRACK_NUM_CDI;

        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_RLT);  // switch to CFAR memory access

        /* get memory offset */
        uint32_t mem_rlt_offset = 0;
        if ((BB_READ_REG(bb_hw, CFR_SIZE_OBJ)) < 256) /* mem_rlt will be splited to 4 banks when cfar size less than 256 */
                mem_rlt_offset = current_frame_type * (1 << SYS_SIZE_OBJ_WD_BNK) * RESULT_SIZE;

        //FIX ME if 2D DoA mode is combined 2d or single shot mode, obj_info_t may be need some change
        volatile obj_info_t *obj_info = (volatile obj_info_t *)(BB_MEM_BASEADDR + mem_rlt_offset);

        init_pk_pos_shft_tbl();

        int rng_index, vel_index;

        for(int obj_ind = 0; obj_ind < obj_num; obj_ind ++)
        {
                rng_index = obj_info[obj_ind].rng_idx;
                vel_index = obj_info[obj_ind].vel_idx; // vel_index is limited within [0, vel_nfft]

                OPTION_PRINT("\n obj_ind = %d \n\n", obj_ind);
                OPTION_PRINT("rng_ind = %d, vel_ind = %d \n", rng_index, vel_index);

                uint32_t FFT_vec_tmp [DDM_CHRP_PHAS_SEQ_PRD*MAX_NUM_RX];
                // Store RXs FFT vector in temporary array: bpm_index 0 : RX0 - RX3, bpm_index 1 : RX0 - RX3, ... bpm_index 3 : RX0 - RX3,
                get_rx_fft_vec(bb, FFT_vec_tmp, rng_index, vel_index);

                float FFT_pow_vec[DDM_CHRP_PHAS_SEQ_PRD];
                for (uint8_t bpm_index = 0; bpm_index < DDM_CHRP_PHAS_SEQ_PRD; bpm_index++) {  // Normally, DDM_CHRP_PHAS_SEQ_PRD equals to cfg->nvarray
                        float fft_power = 0;
                        for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                                /* function cfl_to_complex() costs too much CPU time using GNU compiling tool chain,
                                 * but can be greatly reduced by using metaware tool chain, then the approximation method
                                 * in calculating peak power is not necessary.
                                 * */
                                // complex_t complex_fft = cfl_to_complex(FFT_vec_tmp[bpm_index*MAX_NUM_RX + ch_index], 14, 1, true, 4, false);
                                complex_t complex_fft = cfl_to_complex1(FFT_vec_tmp[bpm_index*MAX_NUM_RX + ch_index]);
                                fft_power += (complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i); // Non-coherent accumulation

                                // An approximation method, use exponent value to represent the power of the CFL type complex data
                                // fft_power += 0x10 - ( 0xf & (FFT_vec_tmp[bpm_index*MAX_NUM_RX + ch_index]) );
                        }
                        FFT_pow_vec[bpm_index] = fft_power;
                        OPTION_PRINT(" FFT_pow_vec[%d] = %5.4f \n", bpm_index, FFT_pow_vec[bpm_index]*FFT_POW_SCAL);
                }

                /* search the true velocity index */
                float max_power = 0; // The power of peaks generated by different TXs
                uint8_t max_sh = 0;  //  Distance between vel_index and the peak generated by TX without chirp phase shifting
                for (uint8_t sh = 0; sh < DDM_CHRP_PHAS_SEQ_PRD; sh ++) {
                        float fft_power = 0;
                        for (uint8_t t = 0; t < DDM_CHRP_PHAS_SEQ_PRD; t++) {
                                OPTION_PRINT(" sh = %d, t = %d ddm_tx_vel_pos[t] = %d \n", sh, t, ddm_tx_vel_pos[t]);
                                if (cfg->tx_groups[t] > 0) {  // Get FFT peak of enabled TXs, ddm_tx_vel_pos[t] only indicates the TX position in velocity domain
                                        uint8_t tmp_ind = (ddm_tx_vel_pos[t] - 1 + sh) % DDM_CHRP_PHAS_SEQ_PRD;
                                        fft_power += FFT_pow_vec[tmp_ind];
                                        OPTION_PRINT(" FFT_pow_vec[%d] = %5.4f \n", tmp_ind, FFT_pow_vec[tmp_ind]*FFT_POW_SCAL);
                                }
                        }
                        if (fft_power > max_power) {
                                max_power = fft_power;
                                max_sh = sh;
                        }
                }
                OPTION_PRINT("tx_on_num[%d] = %d, DDM_CHRP_PHAS_SEQ_PRD = %d \n", current_frame_type, tx_on_num[current_frame_type], DDM_CHRP_PHAS_SEQ_PRD);
                if (tx_on_num[current_frame_type] < DDM_CHRP_PHAS_SEQ_PRD)  // Not to arrange fft_vec when all TXs are on such as when doing ant_calib
                {
                        ddm_pk_pos_shft_tbl[obj_ind] = max_sh; // Don't write true velocity to CFAR memory, this will affect DOA fetching, record shift distance instead.
                        OPTION_PRINT(" ddm_pk_pos_shft[%d] = %d \n", obj_ind, ddm_pk_pos_shft_tbl[obj_ind]);

                        arrange_rx_fft_vec(bb, FFT_vec_tmp, rng_index, vel_index, max_sh);
                }

                get_rx_fft_vec(bb, FFT_vec_tmp, rng_index, vel_index); // Print and check FFT memory result


        }
        baseband_switch_mem_access(bb_hw, old);
}

/*
 * Configure registers for periodically chirp phase shifting, for details refer to
 * section 4.3.9 Programmable Commands for Chirps in Calterah ALPS Radar Baseband User Guide Release 1.0.1
 * */
void fmcw_radio_DDM_cmd_cfg(fmcw_radio_t *radio)
{
        sensor_config_t *cfg = (sensor_config_t *)CONTAINER_OF(radio, baseband_t, radio)->cfg;
        OPTION_PRINT("\n radio->frame_type_id = %d \n", radio->frame_type_id);
        uint8_t frame_type = radio->frame_type_id;

        uint8_t old_bank = fmcw_radio_switch_bank(radio, 0);

        // switch to bank 5 without return value
        // with switching bank here, following RADIO_WRITE_BANK_REG(5, #reg_name, #reg_val) will
        // configure switched bank, not bank 5.
        fmcw_radio_switch_bank(radio, 5 + radio->frame_type_id);

        tx_on_num[frame_type] = 0;
        for(uint8_t tx = 0; tx < MAX_NUM_TX; tx++) {
                if (cfg->tx_groups[tx] != 0) {
                        tx_on_num[frame_type] ++;
                }
        }

        OPTION_PRINT("\n cfg->nvarray = %d \n", cfg->nvarray);
        OPTION_PRINT("\n tx_on_num[%d] = %d \n", frame_type, tx_on_num[frame_type]);
        //uint8_t cmd_num = 1 + ((tx_on_num==1)? 2:(tx_on_num - 1)*2); // 1: number of bank switch CMD; 2: number of TX phase config CMD, I and Q respectively
        uint8_t cmd_num = 1 + (4 - 1)*2;  // By default, all TXs' phase shifters are on, though not all TXs power on.
        uint8_t cmd_prd = DDM_CHRP_PHAS_SEQ_PRD;
        OPTION_PRINT("\n cmd_num = %d \n", cmd_num);
        OPTION_PRINT("\n DDM_CHRP_PHAS_SEQ_PRD = %d \n", DDM_CHRP_PHAS_SEQ_PRD);

        RADIO_WRITE_BANK_REG(5, FMCW_CMD_PRD_1, cmd_prd); // Configure the chirp command period as the phase shifting period

        /* CMD1 config */
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_0, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_1, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_2, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_X_1_3, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_NUM_1_0    , 0x0 );

        /* CMD2 config */
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_0, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_1, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_2, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Y_1_3, 0x0 );
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_NUM_1_1    , 0x0);

        /* CMD3 config */
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_0, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >>  0) & 0xff);
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_1, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >>  8) & 0xff);
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_2, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >> 16) & 0xff);
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_TIMER_Z_1_3, (((radio->up_cycle - CMD_CYCLE_MARGIN) / 2) >> 24) & 0xff);
        RADIO_WRITE_BANK_REG(5, FMCW_CMD_NUM_1_2, cmd_num);

        if (radio->frame_type_id > 0) // Only config bank4 when frame type is 0,
        {                             // different frame types share the same cmd stored in bank4
                fmcw_radio_switch_bank(radio, old_bank);
                return;
        }
        //switch to bank 4 without return value
        fmcw_radio_switch_bank(radio, 4);

        //Configure phase shifter registers in one period
        uint32_t bank4_addr = RADIO_BK4_CPU_ADDR1;
        for (uint8_t cmd_prd_ind = 0; cmd_prd_ind < cmd_prd; cmd_prd_ind ++) {
                fmcw_radio_reg_write(radio, bank4_addr++, 0x00);
                fmcw_radio_reg_write(radio, bank4_addr++, 0x01);   // TX phase shifter registers are in radio bank 1
                        if (ddm_tx_vel_pos[0] > 1) {
                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH0_TX_TUNE0);
                                fmcw_radio_reg_write(radio, bank4_addr++, (TXs_phase_seq[ddm_tx_vel_pos[0] - 2][cmd_prd_ind]>>8)&0xff);

                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH0_TX_TUNE1);
                                fmcw_radio_reg_write(radio, bank4_addr++, TXs_phase_seq[ddm_tx_vel_pos[0] - 2][cmd_prd_ind]&0xff);
                                OPTION_PRINT("\n config cfg->tx_groups[0],  TXs_phase_seq[%d][%d] = %x\n", ddm_tx_vel_pos[0] - 2, cmd_prd_ind, TXs_phase_seq[ddm_tx_vel_pos[0] - 2][cmd_prd_ind]);
                        }

                        if (ddm_tx_vel_pos[1] > 1) {
                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH1_TX_TUNE0);
                                fmcw_radio_reg_write(radio, bank4_addr++, (TXs_phase_seq[ddm_tx_vel_pos[1] - 2][cmd_prd_ind]>>8)&0xff);

                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH1_TX_TUNE1);
                                fmcw_radio_reg_write(radio, bank4_addr++, TXs_phase_seq[ddm_tx_vel_pos[1] - 2][cmd_prd_ind]&0xff);
                                OPTION_PRINT("\n config cfg->tx_groups[1],  TXs_phase_seq[%d][%d] = %x\n", ddm_tx_vel_pos[1] - 2, cmd_prd_ind, TXs_phase_seq[ddm_tx_vel_pos[1] - 2][cmd_prd_ind]);
                        }

                        if (ddm_tx_vel_pos[2] > 1) {
                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH2_TX_TUNE0);
                                fmcw_radio_reg_write(radio, bank4_addr++, (TXs_phase_seq[ddm_tx_vel_pos[2] - 2][cmd_prd_ind]>>8)&0xff);

                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH2_TX_TUNE1);
                                fmcw_radio_reg_write(radio, bank4_addr++, TXs_phase_seq[ddm_tx_vel_pos[2] - 2][cmd_prd_ind]&0xff);
                                OPTION_PRINT("\n config cfg->tx_groups[2],  TXs_phase_seq[%d][%d] = %x\n", ddm_tx_vel_pos[2] - 2, cmd_prd_ind, TXs_phase_seq[ddm_tx_vel_pos[2] - 2][cmd_prd_ind]);
                        }

                        if (ddm_tx_vel_pos[3] > 1) {
                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH3_TX_TUNE0);
                                fmcw_radio_reg_write(radio, bank4_addr++, (TXs_phase_seq[ddm_tx_vel_pos[3] - 2][cmd_prd_ind]>>8)&0xff);

                                fmcw_radio_reg_write(radio, bank4_addr++, RADIO_BK1_CH3_TX_TUNE1);
                                fmcw_radio_reg_write(radio, bank4_addr++, TXs_phase_seq[ddm_tx_vel_pos[3] - 2][cmd_prd_ind]&0xff);
                                OPTION_PRINT("\n config cfg->tx_groups[3],  TXs_phase_seq[%d][%d] = %x\n", ddm_tx_vel_pos[3] - 2, cmd_prd_ind, TXs_phase_seq[ddm_tx_vel_pos[3] - 2][cmd_prd_ind]);
                        }

                OPTION_PRINT("\n bank4_addr = %d \n", bank4_addr );
        }

        fmcw_radio_switch_bank(radio, old_bank);
}



#endif
