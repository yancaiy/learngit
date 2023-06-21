#ifndef SW_TRACE_H
#define SW_TRACE_H

#ifdef USE_SW_TRACE
void trace_info(uint32_t value);
#else
static inline void trace_info(uint32_t value)
{
}
#endif

#endif
