#ifndef ARC_WDG_H
#define ARC_WDG_H


#define EMU_SAVE_WDG_TO_MEM_ADDR       (0x7f0100)
#define PERIOD_TO_MS                   (3 * 100000)

typedef enum {
	WDG_EVENT_TIMEOUT = 0,
	WDG_EVENT_INT,
	WDG_EVENT_RESET
} wdg_event_t;

extern bool arc_wdt_on_flag;

void arc_wdg_init(wdg_event_t event, uint32_t period);
void arc_wdg_deinit(void);
void arc_wdg_update(uint32_t period);
void arc_wdg_status_clear(void);
uint32_t arc_wdg_count(void);
int32_t arc_wdg_feed_wdt(void);
void arc_wdg_check_emu_error_code(void);
void arc_wdg_fault_injection(void);

#endif
