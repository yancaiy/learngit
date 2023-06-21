#include <math.h>
#include "embARC_toolchain.h"
#include "clkgen.h"
#include "sensor_config.h"
#include "baseband_task.h"
#include "baseband.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "arc_exception.h"
#include "alps_hardware.h"
#include "embARC_debug.h"
#include "baseband_dpc.h"
#include "baseband_cas.h"
#include "cascade.h"
#include "baseband_alps_FM_reg.h"
#include "bb_flow.h"
#include "track_cli.h"
#include "radardsp.h"
#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif

#define BB_WRITE_REG(bb_hw, RN, val) baseband_write_reg(bb_hw, BB_REG_##RN, val)

static uint8_t old_bb_clk;
extern QueueHandle_t queue_fix_1p4;
extern SemaphoreHandle_t mutex_frame_count;
int32_t frame_count = -1;
static bool bb_clk_restore_en = false;
/* task variables */
extern SemaphoreHandle_t mutex_initial_flag;
bool initial_flag = true;

void bb_clk_switch()
{
        bb_clk_restore_en = true;
        old_bb_clk = raw_readl(REG_CLKGEN_DIV_AHB);
        bus_clk_div(BUS_CLK_100M); /* when dumping debug data(no buffer), bb clock should be switched to dbgbus clock(100MHz) */
}

void bb_clk_restore() /* After dumping sample debug data(no buffer), bb clock should be restored to default 200MHz */
{
        if (bb_clk_restore_en == true) {
                bb_clk_restore_en = false;
                raw_writel(REG_CLKGEN_DIV_AHB, old_bb_clk);
        }
}

void frame_count_ctrl(void)
{
        xSemaphoreTake(mutex_frame_count, portMAX_DELAY);
        if (frame_count > 0) {
                frame_count--;
        }
        xSemaphoreGive(mutex_frame_count);
}

void initial_flag_set(bool data)
{
        xSemaphoreTake(mutex_initial_flag, portMAX_DELAY);
        initial_flag = data;
        xSemaphoreGive(mutex_initial_flag);
}

#if FRAME_CYCLE_TIME_CHECK
/****************************************************************
;
; This routine:
;       Check whether the BB frame period is as expected
; arg:
;		void
; return:
;       void
; Change tracking:
;       Ver1.0.0 :
;***************************************************************/
void frame_cycle_time_check(void)
{
    sensor_config_t *cfg = sensor_config_get_cur_cfg();
    static uint32_t last_frame_time = 0;
    uint32_t cur_time = xTaskGetTickCount();

    if ((last_frame_time) && (cur_time > last_frame_time) &&
        ((cur_time - last_frame_time) > 1000U / cfg->track_fps)) {
        EMBARC_PRINTF("Frame period[%d] longer than expected[%d]!\r\n",
            cur_time - last_frame_time, 1000U / cfg->track_fps);
    }
    last_frame_time = xTaskGetTickCount();
}
#endif

void baseband_task(void *params)
{
        uint32_t event = 0;
        uint32_t event_bits = 0;

        baseband_t* bb = baseband_get_cur_bb();
        baseband_data_proc_t* dpc = baseband_get_dpc();

#ifdef CHIP_CASCADE
        baseband_cascade_handshake();
        baseband_dc_calib_init(NULL, false, false); // dc_calib after handshake
#endif

        if (DSP_radarDspInit() != E_OK)
        {
            EMBARC_PRINTF("RSP_radarDspInit() failed\r\n");
            return; 
        }
#if HIL_FUNC
        /* [1] ADC data HIL mode */
        /* Radar signal process Task */
        RSP_radarSignalProcessTask();

#else
        /* [2] Orignal normal mode */
        while(1) {
                if (frame_count != 0) {
                        if(track_is_ready(bb->track)) {
                                track_lock(bb->track);
#if FRAME_CYCLE_TIME_CHECK
                                /* Check whether the BB frame period is as expected */
                                frame_cycle_time_check();
#endif
#ifdef FUNC_SAFETY
                                /* bb frame start flag, used for functional-safety SM1 set part handler */
                                bb_frame_start_flag = true;
#endif
                                /* Clear event bit before bb start */
                                event_bits = baseband_clear_event(EVENT_SAM_DONE | EVENT_BB_DONE);
                                if( event_bits != E_OK)
                                {
                                        EMBARC_PRINTF("Event bit set to %d before start @%s", event_bits, __func__);
                                }

                                /* run baseband data proc chain till the end */
                                int index = 0;

                                while (!baseband_data_proc_run(&dpc[index++])) {
                                        ;
			                    }
                                bb = baseband_get_rtl_frame_type();

                                EMBARC_PRINTF("-------------- baseband_task -----------\n");

//                                if( initial_flag == true ) {
//                                        track_pre_start(bb->track);
//                                        initial_flag_set(false);
//                                } else {
//                                        track_run(bb->track);
//                                }
                                /* wait queue for last BB HW run*/
                                BASEBAND_ASSERT(baseband_wait_bb_done(INT_MODE, DEFAULT_TIMOUT) == E_OK);
#ifdef CHIP_CASCADE
                                if (chip_cascade_status() == CHIP_CASCADE_SLAVE) {
                                        /* rx "scan stop" to release spi underlying buffer, must before baseband_write_cascade_ctrl*/
                                        baseband_scan_stop_rx(CMD_RX_WAIT);

                                        baseband_write_cascade_ctrl();
                                } else {
                                        mst_slv_fram_cnt_chek();
                                }
                                fram_cnt_inc();
#endif
                                frame_count_ctrl();
#if BB_INTERFERENCE_CHECK
#ifndef CHIP_CASCADE
                                /*interference identification*/
                                check_interference(bb);
#endif
#endif
                                /* read result from baseband */
                                track_read(bb->track);
                                if (frame_count == 0) {
                                        if (get_track_cfg() == CAN_INT) {
                                                /* last frame need to add delay to avoid can intr send busy
                                                * now delay one frame period to ensure avoid intr clash */
                                                chip_hw_mdelay(50U);
                                        }
//                                        track_output_print(bb->track);/* print the last frame */


                                        baseband_stop(bb);
                                        EMBARC_PRINTF("<EOF>\r\n");
#ifdef FUNC_SAFETY
                                        /* dynamic sample adc flag, used to stop SM11 and SM12 under functional-safety mode */
                                        sample_adc_running_flag = false;
#endif
                                }

                                baseband_workaroud(&bb->bb_hw);
#ifdef FUNC_SAFETY
                                /* functional-safety process */
                                // func_safety_process(NULL);
                                fusa_run_periodic_items_default();
#endif
                        }
                } else  {
                        if (false == initial_flag) {
                                baseband_stop(bb); /* re-call baseband_stop if not asserted in xQueueReceive */
		      }
#ifdef CHIP_CASCADE
                        if(chip_cascade_status() == CHIP_CASCADE_SLAVE)
                                baseband_read_cascade_cmd(0); /* waiting for master command "scan start/stop" */
#endif

                        xQueueReceive(queue_fix_1p4, &event, 1); // add one tick to disturb the idle task loop
                        taskYIELD();
                } /* end if */
        } /* end while */
#endif
}

