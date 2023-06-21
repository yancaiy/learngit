#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"

#include "arc_mpu.h"
#include "mpu_table.h"

ARC_MPU_REGION gst_mpu_cfg[] = {
    ARC_MPU_REGION_ENTRY("DCCM", ARC_DCCM_BASE, ARC_DCCM_SIZE, ARC_MPU_REGION_ALL_ATTR),
};

uint32_t g_mpu_cfg_cnt = sizeof(gst_mpu_cfg)/sizeof(ARC_MPU_REGION);
ARC_MPU_REGION *p_mpu_cfg = gst_mpu_cfg;


void mpu_setup(void)
{
    uint32_t idx = 0U;

    arc_mpu_disable();

    if (g_mpu_cfg_cnt <= ARC_FEATURE_MPU_REGIONS) {
        for(idx = 0;idx < g_mpu_cfg_cnt; idx++) {
            arc_mpu_region(idx, p_mpu_cfg[idx].base, p_mpu_cfg[idx].size,  p_mpu_cfg[idx].attr);
        }
    } else {
        EMBARC_PRINTF("[mpu_setup] mpu table exceed the core cfg, Err!\r\n");
    }

    /* disable other regions */
    for(;idx < REGION_BUTT; idx++) {
        //mpu_region_disable(idx);
    }

    arc_mpu_enable();
    return;
}

