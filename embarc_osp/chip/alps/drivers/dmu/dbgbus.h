#ifndef DBGBUS_H
#define DBGBUS_H

#define USE_50PIN_DCK_BOARD   0  //1:50pin,0:80pin

void dbgbus_input_config(void);
void dbgbus_dump_enable(uint8_t dbg_src);
void dbgbus_hil_ready(void);
void dbgbus_dump_reset(void);
void dbgbus_dump_done(void);
void dbgbus_dump_disable(void);
void dbgbus_free_run_enable(void);
void dbgbus_dump_start(uint8_t dbg_src);
void dbgbus_dump_stop(void);
void gpio_free_run_sync(void);
void dump_reset(void);
void dump_done(void);

#endif
