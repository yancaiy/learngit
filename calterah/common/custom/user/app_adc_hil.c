
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "app_adc_hil.h"
#include "semphr.h"
#include "sharedVar.h"
#include "status.h"

extern float gHILPara[3];

static appAdcOrHilParams adcOrHilFuncParams;

static void appHilFuncGpioModeCfgDpc(baseband_hw_t *bb_hw, hilFuncMode hilMode);


/**************** ADC_DUMP & HIL Function *****************/
bool appCheckIfAdcOrHilFuncNeedProc(void)
{
	if((adcOrHilFuncParams.workState == ADC_HIL_FUNC_STOP) && (adcOrHilFuncParams.frameNum == 0))
    {
		return false;
	}
	else
	{
		return true;
	}
}

int32_t appAdcOrHilFuncParamSetupProc(appAdcOrHilId funcId, uint32_t frameNum, uint8_t param)
{
	int32_t ret = E_OBJ;
	
	if((adcOrHilFuncParams.frameNum > 0) && (adcOrHilFuncParams.workState != ADC_HIL_FUNC_STOP))
	{
		EMBARC_PRINTF("Func[%s] is busy now\r\n", __func__);
	}

	if(frameNum > 0)
	{
		adcOrHilFuncParams.funcId = funcId;
		adcOrHilFuncParams.frameNum = frameNum;
		adcOrHilFuncParams.workState = ADC_HIL_FUNC_START;
		adcOrHilFuncParams.param = param;
		EMBARC_PRINTF("Func[%s] FuncId[%d] frameNum[%d] param[0x%x]\r\n", __func__, funcId, frameNum, param);

		return E_OK;
	}
	else
	{
	    /* frameNum == 0*/
		adcOrHilFuncParams.workState = ADC_HIL_FUNC_STOP;
		adcOrHilFuncParams.frameNum = 0;

		return E_OK;
	}

	return E_OBJ;
}

uint32_t appAdcOrHilFuncWorkStateSwitchProc(void)
{
	baseband_t    *bb = NULL;
	baseband_hw_t *bb_hw = NULL;
	bool     hil_mux = HIL_GPIO;//0:AHB 1:GPIO
	uint32_t dump_position = 0;
	
	// Start ADC_DUMP or HIL
	if((adcOrHilFuncParams.frameNum > 0) && (adcOrHilFuncParams.workState == ADC_HIL_FUNC_START))
	{
		switch(adcOrHilFuncParams.funcId)
		{
			/******* HIL FUNC START PROC *******/  
			//refer to can_hil_signal
			case ADC_HIL_FUNC_HIL:
			{
				//NOTE: refer to [baseband_data_proc_hil]
				baesband_frame_interleave_cnt_clr();//从frame 0重新开始
				//设置控制参数
				adcOrHilFuncParams.profileNum = ADC_HIL_PROFILE_MAX_NUM;	//TBD		
				adcOrHilFuncParams.profileIdx = 0; //start from profile0
				adcOrHilFuncParams.workState = ADC_HIL_FUNC_RUNNING;
				adcOrHilFuncParams.hilFrameInterval = 0;
				adcOrHilFuncParams.procFrameNum = 1;

				//启动 注意dpc被改变 且引脚切换到DMU管理
				appHilFuncGpioModeCfgDpc(bb_hw, adcOrHilFuncParams.param);
				EMBARC_PRINTF("Hil Function Start, Frame[%d] profileIdx[%d] profileNum[%d] dmpMux[%d]\r\n", 
					adcOrHilFuncParams.frameNum, adcOrHilFuncParams.profileIdx, adcOrHilFuncParams.profileNum, 
					adcOrHilFuncParams.param);

			    break;
			}
			default:
			    break;
		}
		
		return ADCHIL_STATU_SW_TO_RUNNING;
	}
	
	// Stop ADC_DUMP or HIL
	if((adcOrHilFuncParams.frameNum == 0) && (adcOrHilFuncParams.workState == ADC_HIL_FUNC_RUNNING))
	{
		switch(adcOrHilFuncParams.funcId)
		{
			/******* HIL FUNC STOP PROC *******/			
			case ADC_HIL_FUNC_HIL:
			{
                bb = baseband_get_rtl_frame_type();
                baseband_stop(bb);
				//appHilFuncPinSwitchToNormal();
				adcOrHilFuncParams.workState = ADC_HIL_FUNC_STOP;
				//recover dsp to normal params 
				EMBARC_PRINTF("Hil Function Stop\r\n");

			    break;
			}
			default:
			    break;
		}

		return ADCHIL_STATU_SW_TO_STOP;
	}

	return ADCHIL_STATU_NO_CHANGE;
}

appAdcOrHilId appAdcOrHilFuncGetFuncId(void)
{
	return adcOrHilFuncParams.funcId;
}

void appAdcOrHilFuncFrameNumDec(void)
{
	adcOrHilFuncParams.frameNum--;
}

uint32_t appAdcOrHilFuncFrameNumGet(void)
{
	return adcOrHilFuncParams.frameNum;
}

void appAdcOrHilFuncProcFrameNumInc(void)
{
	adcOrHilFuncParams.procFrameNum++;
}

uint32_t appAdcOrHilFuncProcFrameNumGet(void)
{
	return adcOrHilFuncParams.procFrameNum;
}

baseband_data_proc_t *appHilFuncBasebandDpcGet(void)
{
	return adcOrHilFuncParams.dpcForHil;
}

void appHilFuncFrameIntervalSet(uint32_t frameInterval)
{
	adcOrHilFuncParams.hilFrameInterval = frameInterval;
	gHILPara[1] = frameInterval/1000.0f;
}

static void appHilFuncGpioModeCfgDpc(baseband_hw_t *bb_hw, hilFuncMode hilMode)
{
    uint16_t bb_ena_0, bb_ena_1, bb_ena_2;
    bool     dpc_end_0 = false, dpc_end_1 = false, dpc_end_2 = false;

	switch (hilMode)
	{
        case HIL_FUNC_MODE_FFT1D:
        {
        }
        case HIL_FUNC_MODE_FFT2D:
        {
        }
        default:
        {
            bb_ena_0 =  SYS_ENA(HIL    , true)
                       |SYS_ENA(FFT_2D , true)
                       |SYS_ENA(CFR    , true)
                       |SYS_ENA(BFM    , true); /* 1st run, HIL(FFT_1D shared), FFT_2D, CFAR and DOA*/
            bb_ena_1 =  0;
            bb_ena_2 =  0;
            break;
        }
    }

    io_mux_dbgbus_dump(); /* gpio mux */
    dbgbus_input_config();

    MDELAY(5);

    if (bb_ena_1 != 0)    /* lvds config */
	    lvds_dump_config(DBG_SRC_DUMP_W_SYNC);
   
    if (bb_ena_1 == 0 && bb_ena_2 == 0)
        dpc_end_0 = true;

    if (bb_ena_1 != 0 && bb_ena_2 == 0)
        dpc_end_1 = true;

    if (bb_ena_2 != 0)
        dpc_end_2 = true;

    /* 1st run */
    adcOrHilFuncParams.dpcForHil[0].pre         = (bb_ena_1 !=0) ? baseband_hil_dump_done : NULL; /* done for FPGA to switch the direction of dbgbus */
    adcOrHilFuncParams.dpcForHil[0].post        = baseband_hil_input_enable;
    adcOrHilFuncParams.dpcForHil[0].post_irq    = baseband_hil_input_disable; /* only run when bb_ena_1 != 0*/
    adcOrHilFuncParams.dpcForHil[0].fi_recfg    = false; //here must be false, changed by liuyj
    adcOrHilFuncParams.dpcForHil[0].stream_on   = false;
    adcOrHilFuncParams.dpcForHil[0].radio_en    = false;
    adcOrHilFuncParams.dpcForHil[0].tx_en       = false;
    adcOrHilFuncParams.dpcForHil[0].sys_enable  =   SYS_ENA(HIL    , true)
						                           |SYS_ENA(FFT_2D , true)
						                           |SYS_ENA(CFR    , true)
						                           |SYS_ENA(BFM    , false);
    adcOrHilFuncParams.dpcForHil[0].cas_sync_en = false;
    adcOrHilFuncParams.dpcForHil[0].sys_irq_en  = BB_IRQ_ENABLE_BB_DONE;
    adcOrHilFuncParams.dpcForHil[0].track_en    = false;  //sdk:false /* enable track at the end of hil */
    adcOrHilFuncParams.dpcForHil[0].end         = true;

	/* 2nd run */
    adcOrHilFuncParams.dpcForHil[1].pre         = NULL;
    adcOrHilFuncParams.dpcForHil[1].post        = NULL;
    adcOrHilFuncParams.dpcForHil[1].post_irq    = NULL;
    adcOrHilFuncParams.dpcForHil[1].fi_recfg    = false;
    adcOrHilFuncParams.dpcForHil[1].stream_on   = false;
    adcOrHilFuncParams.dpcForHil[1].radio_en    = false;
    adcOrHilFuncParams.dpcForHil[1].tx_en       = false;
    adcOrHilFuncParams.dpcForHil[1].sys_enable  = SYS_ENA(BFM    , true);
    adcOrHilFuncParams.dpcForHil[1].cas_sync_en = false;
    adcOrHilFuncParams.dpcForHil[1].sys_irq_en  = BB_IRQ_ENABLE_BB_DONE;
    adcOrHilFuncParams.dpcForHil[1].track_en    = true; //sdk:true /* enable track at the end of hil */
    adcOrHilFuncParams.dpcForHil[1].end         = true;
 
	return;
}

