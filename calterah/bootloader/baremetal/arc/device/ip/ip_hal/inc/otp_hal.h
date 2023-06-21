/****************************************************************
*                                                               *
* FILE NAME: otp_hal.h                                              *
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

#ifndef OTP_HAL_H
#define OTP_HAL_H


/*============================================
;	         Macro Definitions
;-------------------------------------------*/
#define ALPS_TEST_CHIP 0
#define ALPS_M0_CHIP 1
#define ALPS_M1_CHIP 2
#define ALPS_M2_CHIP 3

/*============================================
;             External Function
;-------------------------------------------*/
/****************************************************************
;
; This routine:
;       get chip version from efuse
; arg:
;		void
; return:
;       0: test chip
;       1:alps_m0 chip
;       2:alps_m1 chip
;       3:alps_m2 chip
;***************************************************************/
int32_t otp_get_chip_ver(void);


#endif // OTP_HAL_H
