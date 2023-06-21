#ifndef TRACE_H
#define TRACE_H

void trace_init(void);
void trace_record_error(uint8_t mod, uint8_t func, uint8_t pos, uint8_t error);

#endif
