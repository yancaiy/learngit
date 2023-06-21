#ifndef CPU_CLOCK_LOCK_DETECTOR_H
#define CPU_CLOCK_LOCK_DETECTOR_H


void cpu_clock_lock_detector_init(void);
void cpu_clock_lock_detector_fault_injection(void);
void check_cpu_clock_lock_detector_error_code(void);

#endif
