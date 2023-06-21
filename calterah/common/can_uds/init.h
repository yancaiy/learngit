#ifndef INIT_H
#define INIT_H

extern uint8_t update_test[8];

void can_uds_init(void);
void timer_expired_callback(void *params);

#endif
