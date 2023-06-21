#ifndef ALPS_EMU_H
#define ALPS_EMU_H

#include "alps_emu_reg.h"
#include "alps_reboot_def.h"

static inline void set_boot_state_done(void)
{
    raw_writel(REG_EMU_BOOT_DONE, 1);
}

static inline uint32_t get_boot_state_flag(void)
{
	return raw_readl(REG_EMU_BOOT_DONE);
}

static inline uint32_t chip_security(void)
{
	return raw_readl(REG_EMU_SEC_STA);
}

static inline void reboot_cause_set(uint32_t cause)
{
	raw_writel(REG_EMU_SPARED_BASE, cause);
}

static inline uint32_t reboot_cause(void)
{
	return raw_readl(REG_EMU_SPARED_BASE);
}

#endif
