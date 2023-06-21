#include "embARC.h"
#include "baseband.h"
#include "baseband_cli.h"
#include "sensor_config.h"

#include "embARC_error.h"

#include "calterah_limits.h"
#include "radio_ctrl.h"
#include "cascade.h"
#include "baseband_cas.h"
#include "baseband_task.h"
#include "apb_lvds.h"
#include "baseband_dpc.h"
#include "dbgbus.h"
#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif
#ifdef UNIT_TEST
#define UDELAY(us)
#else
#define UDELAY(us)  chip_hw_udelay(us);
#endif

#define STREAM_ON_EN 1

#define NVE_LEN 256

extern SemaphoreHandle_t mutex_frame_count;
extern int32_t frame_count;

#ifdef CHIP_CASCADE
extern QueueHandle_t queue_cas_sync;
static bool cas_sync_flag = false;
void baseband_cascade_sync_wait(baseband_hw_t *bb_hw);
void baseband_cascade_sync_init();
#define SHAKE_CODE 0xABCD
#endif

#if BB_INTERFERENCE_CHECK
static void read_nve_dat(baseband_t *bb, float * nve_data,int startindex,int endindex);
static int totalone(uint16_t mask);
static bool baseband_interference_detection(baseband_t *bb);
#endif

static baseband_t baseband[MAX_FRAME_TYPE];
static uint8_t current_frame_type = 0;

static bool     fi_en  = true; /* Frame InterLeaving reconfig enable */
static uint32_t fi_cnt = 0;    /* Frame InterLeaving reconfig count */

static frame_intrlv_t fi        = FI_ROTATE;
static frame_intrlv_t fi_return = FI_ROTATE; /* save the initial strategy */

#if BB_INTERFERENCE_CHECK
static float    nve_data_previous[NVE_LEN];
static float    nve_out[NVE_LEN];
static float    nve_out_buff[NVE_LEN];/*for NVE calculation */
#endif

void baseband_dump_stop(void);

int32_t baseband_init(baseband_t *bb)
{
        int32_t status = E_OK;

        // Duplicate ID to ease access
        bb->radio.frame_type_id = bb->bb_hw.frame_type_id;
        status = fmcw_radio_init(&bb->radio);
        if (status == E_OK) {

        radar_param_config(&bb->sys_params);
        baseband_hw_init(&bb->bb_hw);
        bb->track = track_get_track_ctrl();
        track_init_sub(bb->track, bb);
        cluster_init(&bb->cluster);
        baseband_data_proc_init();
#ifdef FUNC_SAFETY
        static int32_t powrer_on_check_status = 0;
        if(powrer_on_check_status == 0)
        {
        /*leave power save mode before sm fault injection, and enter power save mode after injection*/
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
                baseband_hw_t *bb_hw = &bb->bb_hw;
                if ((bb_hw->frame_type_id) == (NUM_FRAME_TYPE - 1)) { /* run one time */
                        powrer_on_check_status++;
                        safety_mechanism_power_on_check(&bb->radio);
                }
#if (SAFETY_FEATURE_SM104 == FEATURE_ON)
                clear_bb_memory_by_scan_start_bb_one_time();
#endif
#if (INTER_FRAME_POWER_SAVE == 1)
                baseband_interframe_power_save_enable(&bb->bb_hw, true);
#endif
        }
#endif // FUNC_SAFETY
        }

        return status;
}

void baseband_clock_init()
{
        bb_enable(1); /* bb clock enable */
}

int32_t baseband_dump_init(baseband_t *bb, bool sys_buf_store)
{
        baseband_hw_dump_init(&bb->bb_hw, sys_buf_store);
        return E_OK;
}

void baseband_start(baseband_t *bb)
{
#if (INTER_FRAME_POWER_SAVE == 1)
        baseband_interframe_power_save_enable(&bb->bb_hw, false);
#endif
        fmcw_radio_sdm_reset(&bb->radio);
        UDELAY(100);

        dmu_adc_reset(); // ADC should reset in cascade

        track_start(bb->track);
        baseband_hw_start(&bb->bb_hw);

#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER)
                baseband_cascade_sync_wait(&bb->bb_hw); // wait for slave ready signal
#endif

        if (!fmcw_radio_is_running(&bb->radio)) {
                fmcw_radio_start(&bb->radio);
        }
#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_SLAVE) {
                cascade_s2m_sync_bb();                  // slave sents ready signal
        }
#endif

}

void baseband_start_with_params(baseband_t *bb, bool fmcw_en, bool tx_en, uint16_t sys_enable, bool cas_sync_en, uint8_t sys_irp_en, bool track_en)
{
        if (fmcw_en == true){
#if INTER_FRAME_POWER_SAVE == 1
                /* Recover the radio setting from power save mode */
                fmcw_radio_adc_ldo_on(&bb->radio, true);
                fmcw_radio_lo_on(&bb->radio, true);
                fmcw_radio_rx_on(&bb->radio, true);
#endif
                fmcw_radio_sdm_reset(&bb->radio);
                UDELAY(100);

                dmu_adc_reset(); // ADC should reset in cascade
        }

        if (track_en == true) {
                track_start(bb->track);
        }

        /* Enable and disable dmu_irq using sys_irp_en */
        dmu_irq_enable(INT_BB_SAM, sys_irp_en & 0x4);
        dmu_irq_enable(INT_BB_DONE, sys_irp_en & 0x1);

        baseband_hw_start_with_params(&bb->bb_hw, sys_enable, BB_IRQ_ENABLE_ALL);

#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_MASTER && cas_sync_en) {
                baseband_cascade_sync_wait(&bb->bb_hw); // wait for slave ready signal
        }
#endif
        if (tx_en == true) {
                fmcw_radio_tx_restore(&bb->radio);
                #if AUTO_TX == 1
                fmcw_radio_tx_auto_ch_on(&bb->radio, -1, true);
                #endif
        }

        if ((!fmcw_radio_is_running(&bb->radio)) && fmcw_en==true) {
                fmcw_radio_start(&bb->radio);
        }

#ifdef CHIP_CASCADE
        if (chip_cascade_status() == CHIP_CASCADE_SLAVE && cas_sync_en) {
                cascade_s2m_sync_bb();                  // slave sents ready signal
         }
#endif

}

void baseband_stop(baseband_t *bb)
{
        initial_flag_set(true);
        /* reset baseband & radio bank to default bank */
        baseband_frame_interleave_cfg(0);
        fmcw_radio_stop(&bb->radio);
        baseband_hw_stop(&bb->bb_hw);
        track_stop(bb->track);

        baseband_dump_stop(); /* stream data dump stop */
        bb_clk_restore();     /* restore bb clock after dumping sample debug data */
        if (true == baseband_data_proc_req()) {
                baseband_data_proc_init(); /* re-init data process chain */
                dmu_hil_input_mux(HIL_AHB); /* trun off dbgbus input */
                dump_done();           /* done signal to  fpga board */
        }

        /* dmu will be switched to GPIO mode at end of dump stop */
        if (io_get_dmumode() != SYS_DMU_SEL_GPIO) {
                io_sel_dmumode(SYS_DMU_SEL_GPIO);
        }
}

baseband_t* baseband_get_bb(uint32_t idx)
{
        baseband_t* ret = NULL;
        if (idx >= NUM_FRAME_TYPE) {
                ret = NULL;
        }
        else {
                ret = &baseband[idx];
        }
        return ret;
}

baseband_t* baseband_get_cur_bb()
{
        sensor_config_t *cfg = sensor_config_get_cur_cfg();
        return cfg->bb;
}

uint8_t baseband_get_cur_idx()
{
        return sensor_config_get_cur_cfg_idx();
}

uint8_t baseband_get_cur_frame_type()
{
        return current_frame_type;
}

uint8_t baseband_get_cur_frame_strategy()
{
        return fi.strategy;
}

void baseband_set_cur_frame_type(uint8_t ft)
{
        current_frame_type = ft;
}

baseband_t* baseband_get_rtl_frame_type()
{
        current_frame_type = baseband_read_reg(NULL, BB_REG_FDB_SYS_BNK_ACT); /* read back the bank selected in RTL */
        baseband_t* bb = baseband_get_bb(current_frame_type);
        baseband_write_reg(&bb->bb_hw, BB_REG_SYS_BNK_ACT, current_frame_type);     /* match the bank accessed by CPU */
        sensor_config_set_cur_cfg_idx(current_frame_type);
        return bb;
}

void baseband_frame_type_reset()
{
        baseband_write_reg(NULL, BB_REG_SYS_BNK_RST,  1);
        fmcw_radio_frame_type_reset(NULL);
}

/* configure the loop pattern in frame interleaving */
baseband_t* baseband_frame_interleave_cfg(uint8_t frame_loop_pattern)
{
        baseband_frame_interleave_pattern(NULL, frame_loop_pattern);
        fmcw_radio_frame_interleave_pattern(NULL, frame_loop_pattern);
        return baseband_get_rtl_frame_type(); /* read back the bank selected in RTL */
}

/* set new strategy in frame interleaving */
void baesband_frame_interleave_strategy_set(uint8_t strategy, uint32_t sw_num, uint8_t sel_0, uint8_t sel_1)
{
                fi.strategy = strategy;       // FIXMED or ROTATE or AIR_CONDITIONER or VELAMB or customized by yourself
                fi.sw_num   = sw_num;         // loop period or switch number
                fi.sel_0    = sel_0;          // 1st frame type used
                fi.sel_1    = sel_1;          // 2nd frame type used
}

/* get current frame interleaving info */
frame_intrlv_t baesband_frame_interleave_strategy_get()
{
        return fi;
}

/* return the default strategy in frame interleaving */
void baesband_frame_interleave_strategy_return()
{
        fi = fi_return;
}

/* clear the fi_cnt in frame interleaving */
void baesband_frame_interleave_cnt_clr()
{
        fi_en  = true;
        fi_cnt = 0;
}

/* reconfigure the loop pattern in frame interleaving, just a reference code here*/
baseband_t* baseband_frame_interleave_recfg()
{
        baseband_t* bb = baseband_get_rtl_frame_type();

        switch (fi.strategy) {

        case FIXED : // fix one frame type
                if (fi_en && fi_cnt == 0) {
                        bb = baseband_frame_interleave_cfg(fi.sel_0);
                }
                break;

        case ROTATE : // alwasys loop all available frame types
                bb = baseband_frame_interleave_cfg(fi_cnt % fi.sw_num);
                break;

        case AIR_CONDITIONER : // use fi.sel_0 first, then fi.sel_1
                if (fi_en) {
                        if (fi_cnt == 0) {
                                bb = baseband_frame_interleave_cfg(fi.sel_0);
                        } 
		      else  {
			    if (fi_cnt == (fi.sw_num - 1)) {
                                	bb = baseband_frame_interleave_cfg(fi.sel_1);
                                	fi_en = false;
  			    }
                        }
                }
                break;

        case VELAMB : // always loop
                if (fi_cnt % fi.sw_num  == 2) {
                        bb = baseband_frame_interleave_cfg(fi.sel_1);
                }
                else {
                        bb = baseband_frame_interleave_cfg(fi_cnt % fi.sw_num);
                }
	       break;

        // you can customize a new pattern here

        default:
	       if (fi_en) {
	           ;
	       }
                break;

        }

        if (fi_en) {
                if (fi_cnt < (fi.sw_num - 1)) {
                        fi_cnt++;
	       }
                else {
                        fi_cnt = 0;
	       }
        }

        return bb;
}

/*parse each element of cfg->tx_groups to work state in a chirp way*/
bool bit_parse(uint32_t num, uint16_t bit_mux[])
{
        bool valid_mux = true;
        for(int i = 0; i< MAX_NUM_TX; i++){
                bit_mux[i] = (num >> (4*i)) & 0x3;   //output tx work status in chirp i, bit_mux=0 tx off; bit_mux=1 tx on in phase; bit_mux=2  tx on opposite phase
                if (bit_mux[i] == 3) {
                        valid_mux = false;
                        break;
                }
        }
        return valid_mux;
}

/*get the phase of the first chirp, when more than 1 tx is on in the first chirp, return the phase of antenna with small index
mainly used to judge the transmittion order of BPM*/
uint16_t get_ref_tx_antenna(uint32_t patten[])
{
        uint16_t refer_phase = 0;
        uint16_t bit_mux[MAX_NUM_TX] = {0, 0, 0, 0};
        for(int i = 0; i < MAX_NUM_TX; i++) {
                bit_parse(patten[i],bit_mux);
                if (bit_mux[0]>0) {
                        refer_phase = bit_mux[0];
                        break;
                }
        }
        return refer_phase;
}

/*to get the tx mux in every chirp by parse all patten*/
bool get_tdm_tx_antenna_in_chirp(uint32_t patten[], uint8_t chip_idx, uint32_t chirp_tx_mux[])
{
        uint16_t bit_mux[MAX_NUM_TX] = {0, 0, 0, 0};
        int32_t tx_groups_tmp[MAX_NUM_TX][MAX_NUM_TX] = {0};
        uint32_t var = 0;
        bool ret = true;

        for(int i = 0; i < MAX_NUM_TX; i++) {          //Tx loop
                bit_parse(patten[i], bit_mux);
                for(int c = 0; c < MAX_NUM_TX; c++) {  //chirp loop
                        if (bit_mux[c] > 0) {
                                tx_groups_tmp[i][c] = 1;  //output in which chirps Txi transmit
                        }
	       }
        }
        for(int c = 0; c <= chip_idx; c++) {  //chirp loop
                var = 0;
                for (int i = 0; i < MAX_NUM_TX; i++) {          //Tx loop
                        if (tx_groups_tmp[i][c] > 0) {
                                var |= (1 << i);
                        }
                }
                if (var == 0) {
		      ret = false;
                         break;     //mean all tx off in chirp c
                }
                chirp_tx_mux[c] = var;    //ret is the mux presentation of txs which are on in chirp c
        }
        return ret;
}

/*check bit_mux of a tx if satisfy the standard bpm pattern
for bpm va = 2, pattern 0 is x x , pattern 1 is x -x
for bpm va = 4, pattern 0 is x x x x, pattern 1 is x -x x -x, pattern 2 is x x -x -x, pattern 3 is x -x -x x*/
int8_t bpm_patten_check(uint16_t bit_mux[], uint8_t chip_idx, uint16_t refer_phase, uint8_t pat_num)
{
        int loop_num = chip_idx/2 + 1;     //for patten 1 2 3 check
        int8_t p = 0;
        for (int m = 0; m <= pat_num; m++ ) {
                p = 0;      //indicate which pattern bit_mux shows. p = -1 :mean invalid pattern
                switch (m) {
                case 0:
                        for(int j=0;j<=chip_idx;j++){
                               if (bit_mux[j] != refer_phase) {
                                       p = -1;
                                       break;
                               }
                        }
                        if (p != -1) {
                                p = 0;         // patten 0 gotten     1 1 1 1
                        }
		      break;
                case 1:
                        for (int j = 0; j < loop_num; j++) {
                                if( (bit_mux[2*j] != refer_phase) || ((bit_mux[2*j] + bit_mux[2*j + 1]) != 3) ){
                                        p = -1;
                                        break;
                                }
                        }
                        if (p != -1) {
                                p = 1;         // patten 1 gotten  1 -1 1 -1
                        }
		      break;
                case 2:
                        for (int j = 0; j < loop_num; j++) {
                                uint16_t phase = (j == 0)?refer_phase:(3-refer_phase);
                                if( (bit_mux[2*j] != phase) || (bit_mux[2*j] != bit_mux[2*j + 1]) ){
                                        p = -1;
                                        break;
                                }
                        }
                        if (p != -1) {
                                p = 2;         // patten 2 gotten  1 1 -1 -1
                        }
		      break;
                case 3:
                       for (int j = 0; j < loop_num; j++) {
                               uint16_t phase = (j == 0)?refer_phase:(3-refer_phase);
                                if( (bit_mux[2*j] != phase) || ( (bit_mux[2*j] + bit_mux[2*j + 1]) != 3) ){
                                        p = -1;
                                        break;
                                }
                       }
                        if (p != -1) {
                                p = 3;         // patten 3 gotten  1 -1 -1 1
                        }
		      break;
                default :
		      if(p) {   //fix M2CM-Rule-16.4(qac-10.1.0-2016)
			  ;
		      }
                        break;
                }
                if (p != -1) {
                        break;
	       }
        }
        return p;
}

int8_t get_bpm_tx_antenna_in_chirp(uint32_t patten[], uint8_t chip_idx, uint32_t chirp_tx_mux[])
{
        uint16_t bit_mux[MAX_NUM_TX] = {0, 0, 0, 0};
        int8_t checker = 0;     //checker = -1: pattern not found
        uint16_t refer_phase = 0;
        int i = 0;
        int32_t tx_groups[MAX_NUM_TX] = {0, 0, 0, 0};
        uint32_t ret = 0;
        refer_phase = get_ref_tx_antenna(patten);
        if (refer_phase == 0) {       //mean all tx off in chirp 0
                checker = -1;
                return checker;
        }
        // get patten 0 first
        for(i = 0; i < MAX_NUM_TX; i++) {
                bit_parse(patten[i],bit_mux);
                checker = bpm_patten_check(bit_mux, chip_idx, refer_phase, 0);
                if (checker==0) {
                        tx_groups[i] = 1;
                        break;
                } else {
                        continue;
                }
        }
        if (checker == -1) {
                return  checker;
        }
        // get other pattern
        for(int j = 0; j < MAX_NUM_TX; j++) {
                if (j == i-1) {
                        continue;
	       }
                bit_parse(patten[j],bit_mux);
                uint8_t patten_num = (chip_idx == 1)?1:3;      //chip_idx == 1 mean varray = 2
                checker = bpm_patten_check(bit_mux, chip_idx, refer_phase, patten_num);
                if (checker!=-1) {
                        tx_groups[j] = checker + 1;
                }
        }
        /*generate the tx mux which are on in the same chirp*/
        for (int c=0; c<= chip_idx; c++) {
                ret = 0;
                for(i = 0; i < MAX_NUM_TX; i++) {
                        if (tx_groups[i] == (c+1)) {
                               ret |= (1<<i);
		      }
                }
                if (ret == 0) {
                        checker = -1;
                        return checker;
                }
                chirp_tx_mux[c] = ret;
        }
        checker = 4;     //success over
        return checker;
}

void baseband_dump_stop(void)
{
        bool param = baseband_stream_off_req();
        if (param == STREAM_ON_EN) { /* data dump finish */
#ifdef CHIP_CASCADE
        lvds_dump_stop();
#else

#if (USE_50PIN_DCK_BOARD == 1)
        /* Do nothing */
#else
        dbgbus_dump_stop();
#endif //USE_50PIN_DCK_BOARD

#endif // CHIP_CASCADE
        baseband_switch_buf_store(NULL, SYS_BUF_STORE_FFT);
        }
}

void baseband_hil_input_enable(baseband_t *bb)
{
        /* change the dck to the tx status */
        dump_done();
        chip_hw_udelay(50);

        dmu_hil_input_mux(HIL_GPIO);
        /* ready signal of hil sent to FPGA data collection board */
        dbgbus_hil_ready();
}

void baseband_hil_input_disable(baseband_t *bb)
{
        dmu_hil_input_mux(HIL_AHB); /* trun off dbgbus input */
}

void baseband_hil_dump_enable(baseband_t *bb)
{
        dump_reset();  /* reset signal to fpga board */
        chip_hw_udelay(50);
        fmcw_radio_lvds_on(NULL, true);
}

void baseband_hil_dump_done(baseband_t *bb)
{
        dump_done();  /* done signal to  fpga board */
}

void baseband_scan_stop(void)
{
        xSemaphoreTake(mutex_frame_count, portMAX_DELAY);
        frame_count = 0;
        xSemaphoreGive(mutex_frame_count);
}

#ifdef CHIP_CASCADE
void baseband_cascade_handshake()
{
        if (chip_cascade_status() == CHIP_CASCADE_MASTER) {
                EMBARC_PRINTF("wait for handshake ...\r\n");

                cascade_read_buf_req(portMAX_DELAY); /* wait buf */
                if (cascade_read_buf(0) == SHAKE_CODE)
                        EMBARC_PRINTF("handshake success!\r\n");

                cascade_read_buf_done();

                // cascade sync init after handshake
                baseband_cascade_sync_init();

        } else {
                EMBARC_PRINTF("tx handshake ...\r\n");

                cascade_write_buf_req();
                cascade_write_buf(SHAKE_CODE);
                cascade_write_buf_done();
        }

}

void baseband_cascade_sync_handler()
{
        uint32_t msg = 0;
        if (cas_sync_flag)
                xQueueSendFromISR(queue_cas_sync, (void *)&msg, 0);

}

void baseband_cascade_sync_wait(baseband_hw_t *bb_hw)
{
        uint32_t event = 0;

        if(xQueueReceive(queue_cas_sync, &event, portMAX_DELAY) == pdTRUE) {
                if (true == baseband_scan_stop_req()) {
                        uint16_t param = baseband_stream_off_req();
                        baseband_scan_stop_tx(CMD_SCAN_STOP, param); /* tx "scan stop" commond to slave*/
                        baseband_scan_stop(); /* scan stop for master itself */
                }
        }

}

void baseband_cascade_sync_init()
{
        cas_sync_flag = true;
}
#endif

#if BB_INTERFERENCE_CHECK
/*
 * Parameters:
 * baseband_t *bb : current frame type baseband configs
 * float * nve_data : to store NVE data read from MEM_NVE
 */
static void read_nve_dat(baseband_t *bb, float * nve_data,int startid,int endid)
{
        baseband_hw_t *bb_hw = &bb->bb_hw;
        uint8_t current_frame_type = baseband_read_reg(NULL, BB_REG_FDB_SYS_BNK_ACT);
        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_NVE);
        // NVE bank start address:0x000, 0x1000, 0x2000, 0x3000
        uint32_t mem_nve_bnk_offset = 1 * (1<<12);  // 2^12 is bank address offset
        uint32_t *mem_nve_ptr = (uint32_t *)(BB_MEM_BASEADDR + current_frame_type * mem_nve_bnk_offset);
        for (int rng_index = startid; rng_index  < endid ; rng_index ++) {

                uint32_t mem_nve_dat = *(mem_nve_ptr + rng_index);
                *(nve_data + rng_index) = fl_to_float(mem_nve_dat , 15, 1, false, 5, false);

        }

        baseband_switch_mem_access(bb_hw, old);
}

/*estimate final nve based on previous nve data and nve from 2dfft data*/
float estimate_nve(baseband_t *bb)
{
        baseband_hw_t *bb_hw = &bb->bb_hw;
        sensor_config_t* cfg = bb->cfg;
        /*calculate NVE value*/
        uint32_t fft_mem;
        uint32_t rx_num = cfg->nvarray * MAX_NUM_RX;
        uint32_t scalar = 10000000;
        uint32_t Rng_start = cfg->fft_nve_rangeindex_start;
        uint32_t Rng_end = cfg->fft_nve_rangeindex_end < NVE_LEN ? cfg->fft_nve_rangeindex_end : NVE_LEN;
        EMBARC_ASSERT(Rng_start < Rng_end);
        /*read previous nve value*/
        read_nve_dat(bb, nve_data_previous,Rng_start,Rng_end);
        float    temp_tone_scalar = 1.2;
        int      max_channel = totalone(cfg->fft_nve_ch_mask);
        int      rx_size = max_channel > rx_num ? rx_num : max_channel;
        float    fft_nve_tone_scalar = temp_tone_scalar / rx_size;
        int      fft_nve_tone_shift = 4;
        float    nve_mean = 0;
        uint32_t old = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
        for (uint32_t tmp_rng_ind = Rng_start; tmp_rng_ind < Rng_end; tmp_rng_ind ++){
                uint32_t Rng_index = tmp_rng_ind-Rng_start;
                uint32_t vel_idx = 0;
                uint32_t cnt = 0;
                float pow_sum[4] = {0};
                float pow_tmp = 0;
                float temp,t1,tmp1,tmp2,t2,minus = 0;
                for(uint32_t t = 0;t < 4;t ++){
                        vel_idx = t * cfg->vel_nfft / 4 + 2;
                        for(uint32_t bpm_index = 0;bpm_index < cfg->nvarray; bpm_index ++){
                                for(uint32_t tmp_rx_ind = 0; tmp_rx_ind < MAX_NUM_RX; tmp_rx_ind ++){
                                        fft_mem = baseband_hw_get_fft_mem(bb_hw, tmp_rx_ind, tmp_rng_ind, vel_idx, bpm_index);
                                        complex_t  complex_fft= cfl_to_complex(fft_mem, 14, 1, true, 4, false);
                                        pow_tmp=complex_fft.r * complex_fft.r + complex_fft.i * complex_fft.i;
                                        pow_sum[t] += pow_tmp;
                                }
                        }
                }
                /*sort pow_sum as from small to big*/
                quicksort(pow_sum,0,3);
                /*average among different channels*/
                for(int vel_idx = 0; vel_idx < 4; vel_idx ++){
                        temp = pow_sum[vel_idx] / fft_nve_tone_shift;
                        if(temp < pow_sum[0]){
                                minus += pow_sum[vel_idx];
                                cnt ++;
                        }
                }
                switch(cnt){
                        case 2:
                                minus = minus / 2;
                                break;
                        case 3:
                                t1 = minus / 4;
                                t2 = minus / 8;
                                minus = t1 + t2;
                                break;
                        case 4:
                                minus = minus / 4;
                        case 1:
                                default:
                                break;
                }
                minus = minus * fft_nve_tone_scalar;
                if (cfg->fft_nve_bypass){
                        nve_out[Rng_index] = cfg->fft_nve_default_value;
                } else {
                        tmp1 = nve_data_previous[tmp_rng_ind] * scalar;
                        tmp2 = minus * scalar;
                        if(tmp1>tmp2 && tmp2!=0){
                                nve_out_buff[Rng_index] = tmp2;
                                nve_out[Rng_index] = 0;
                        } else {
                                nve_out_buff[Rng_index] = tmp1;
                                nve_out[Rng_index] = 0;
                        }
                }
        }
        /*moving average*/
        if(!(cfg->fft_nve_bypass)){
                int mov_len = 2;
                int mov_total = 2 * mov_len + 1;
                for(uint32_t tmp_rng_ind = 0; tmp_rng_ind < Rng_end - Rng_start; tmp_rng_ind ++){
                       if(mov_len == 0 || tmp_rng_ind < mov_len || Rng_end - Rng_start - tmp_rng_ind <mov_len){
                                nve_out[tmp_rng_ind] = nve_out_buff[tmp_rng_ind];
                       } else {
                                for(int i = -mov_len; i <mov_len; i ++){
                                        nve_out[tmp_rng_ind] += nve_out_buff[tmp_rng_ind + i] / mov_total;
                                }
                        }
                /*get the mean value of nve*/
                        nve_mean += nve_out[tmp_rng_ind] / (Rng_end-Rng_start + 1);
                }
        }
        baseband_switch_mem_access(bb_hw, old);
        return nve_mean;
}
/*for calculating ones in mask*/
static int totalone(uint16_t mask)
{
    int count = 0;
    while(mask){
        count += mask & 0x1;
        mask >>= 1;
    }
    return count;
}


/* ================================baseband interference detection function definition =============================== */
/*
 * Parameters:
 * baseband_t *bb : current frame type baseband configs
 * output: result of intereference detection
 * */
static bool baseband_interference_detection(baseband_t *bb)
{
        sensor_config_t* cfg = bb->cfg;
        float nve_estimation = estimate_nve(bb);
        /*judge whether there is interference or not */
         if(nve_estimation > ((cfg->fft_nve_without_interference) * (cfg->fft_nve_threash))){
             return true;
        } else {
             return false;
        }
}

/*
 * Parameters:
 * baseband_t *bb : current frame type baseband configs
 * output: result of intereference detection
 * */
void check_interference(baseband_t * bb)
{
   baseband_interference_detection(bb);
}
#endif // BB_INTERFERENCE_CHECK

