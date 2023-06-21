/****************************************************************
*                                                               *
* FILE NAME: otp_hal.c											*
*                                                               *
*---------------------------------------------------------------*
* COMPILER:                                  arc_gnu_2018.03    *
*                                    mw_safety_arc_N-2018_03    *
*---------------------------------------------------------------*
* REVISION HISTORY:                                             *
*                                                               *
*===============================================================*
*    Copyright (c) 2017-2021 Calterah Semiconductor Technology  *
*                                          All rights reserved. *
****************************************************************/

#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include "otp_hal.h"


/*============================================
;             External Function
;-------------------------------------------*/
/****************************************************************
;
; This routine:
;       get chip version from efuse
; arg:
;       void
; return:
;       0:alps_test chip
;       1:alps_m0 chip
;       2:alps_m1 chip
;       3:alps_m2 chip
;***************************************************************/
int32_t otp_get_chip_ver(void)
{
    uint32_t result = E_OK;
    uint32_t reg = OTP_CHIP_VER;

    if ((raw_readl(reg) >> 23) & 0x1){
        result = 0;
    }else{
        result = (raw_readl(reg) >> 16) & 0x7f;
    }

    return result;
}
