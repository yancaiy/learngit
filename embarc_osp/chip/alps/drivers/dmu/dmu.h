#ifndef DMU_H
#define DMU_H

#include "dbgbus.h"

void dmu_hil_ahb_write(uint32_t hil_data);
bool dmu_hil_input_mux(bool input_mux);

void dmu_adc_reset(void);

#endif
