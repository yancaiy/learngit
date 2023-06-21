#ifndef MPU_TABLE_H
#define MPU_TABLE_H

#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"

#define ARC_DCCM_BASE  0xA00000
#define ARC_DCCM_SIZE  0x8000


enum MPU_REGION_NUMBER
{
    REGION_0  = 0,
    REGION_1,
    REGION_2,
    REGION_3,
    REGION_4,
    REGION_5,
    REGION_6,
    REGION_7,
    REGION_8,
    REGION_9,
    REGION_10,
    REGION_11,
    REGION_12,
    REGION_13,
    REGION_14,
    REGION_15,

    REGION_BUTT,
};

#endif
