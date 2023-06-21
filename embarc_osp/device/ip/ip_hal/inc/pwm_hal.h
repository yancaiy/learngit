#ifndef PWM_HAL_H
#define PWM_HAL_H

int32_t pwm_init(void);

int32_t pwm_start(uint32_t id, uint32_t freq, uint32_t duty_cycle, void (*func)(void *param));

int32_t pwm_stop(uint32_t id);
#endif
