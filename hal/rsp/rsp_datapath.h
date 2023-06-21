/**
 * @file     rsp_datapath.h
 * @brief    This file defines the functions to configure and run the dataPath of RSP(Radar Signal Process) in radar device.
 *           And use the RSP dataPath interface to get the data or output them from dataPath in RSP.
 * @author   Wison (linkangcheng@chengtech.net)
 * @version  1.0
 * @date     2022-09-27
 * 
 * 
 * @par Verion logs:
 * <table>
 * <tr>  <th>Date        <th>Version  <th>Author     <th>Description
 * <tr>  <td>2022-09-27  <td>1.0      <td>Wison      <td>First Version
 * </table>
 * @copyright Copyright (c) 2022  Shenzhen Cheng-Tech Co.,Ltd.
 */

#ifndef _RSP_DATAPATH_H_
#define _RSP_DATAPATH_H_


/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "cfg.h"
#include "sharedVar.h"
#include "radardsp.h"


void setFrameFlag(uint32_t data);
void DSPConfigure(const fmcwParam_t *param);
Bin_Param_t getBinParam(void);
void DSPConfigSet(uint8_t track_frame_type, int32_t *tx_profile, int32_t *chirp_type, int32_t *tx_mask_flag);

void RSP_dataProcess_mergeTargets(
    Target_Thres_t (*inTarget)[2][MAX_NB_OF_TARGETS],
    radarDspParam_t *dspParam,
    objectInfo_t *outObject);
void RSP_dataProcess_fixVelocity(
    Target_Thres_t (*inTarget)[2][MAX_NB_OF_TARGETS],
    radarDspParam_t *dspParam,
    objectInfo_t *outObject,
    int tx,
    int chirp);

int RSP_radarSignalVarInit(void);
void RSP_radarSignalProcessTask(void);

#endif

