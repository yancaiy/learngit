#ifndef STATISTIC_H
#define STATISTIC_H

#define STAT_TIMER_ID               TIMER_1
#define STAT_TIMER_INTNO            INTNO_TIMER1

extern volatile uint64_t cpu_run_time;
uint32_t stat_timer_init();
void cpu_load_generator_task(void *params);

#endif