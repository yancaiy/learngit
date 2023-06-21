
#ifndef __STATUS_H__
#define __STATUS_H__

#include "typedefs.h"

#define RTC_TO_US(rtc)  ((rtc) / 300)
#define RTC_TO_MS(rtc)  (RTC_TO_US(rtc) / 1000)

#define US_TO_RTC(us)   ((us) * 300)
#define MS_TO_RTC(ms)   (US_TO_RTC(ms) * 1000)


extern void setTimestamp(uint64_t timestamp);
extern void updateTimestamp(void);
extern uint64_t getTimestamp(void);
extern void setCyroCaliFlag(uint8_t flag);
extern uint8_t getCyroCaliFlag(void);

#endif

