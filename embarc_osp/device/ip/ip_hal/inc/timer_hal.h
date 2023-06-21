#ifndef TIMER_HAL_H
#define TIMER_HAL_H

int32_t dw_timer_init(void);
int32_t dw_timer_mode_config(uint32_t id,uint32_t mode);
int32_t dw_timer_start(uint32_t id, uint32_t expire, void (*func)(void *param),void *param);
int32_t dw_timer_stop(uint32_t id);
int32_t dw_timer_current(uint32_t id, uint32_t *current);

#endif
