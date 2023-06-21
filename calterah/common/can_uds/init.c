#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "init.h"

#include "can_config.h"

uint8_t update_test[8] = {'2','0','2','0','1','0','2','1'};

void timer_expired_callback(void *params)
{
        //EMBARC_PRINTF("timer expired!\r\n");
        /* can transport state. */
        cantp_state_main();

        /* uds state & data handle. */
        uds_sd_state_main();
}

void can_uds_init(void)
{
        canif_init();
        cantp_init();
        canif_callback_register(0, cantp_confirmation, 0, cantp_indication);

        uds_dispatch_init();

        //can_callback_register(dev_can->id, canif_rx_indication, canif_tx_confirmation);
        //can_enable_controller_interrupt(dev_can->id);
        //can_enable_controller_interrupt(dev_can->id);
#ifndef OS_FREERTOS
        extern void tick_callback_register(void (*func)(void *));
        tick_callback_register(timer_expired_callback);
#endif
}

