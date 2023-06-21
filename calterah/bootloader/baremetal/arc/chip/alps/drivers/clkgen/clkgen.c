#include "embARC_toolchain.h"
#include "embARC_error.h"

#include "clkgen.h"
#include "radio_ctrl.h"
#include "arc_builtin.h"
#include "arc.h"
#include "arc_exception.h"
#include "alps_clock_reg.h"
#include "alps_clock.h"
#include "alps_module_list.h"
#include "alps_timer.h"
#include "embARC_debug.h"
#include "alps_emu.h"
#include "calterah_error.h"

int32_t clock_select_source(clock_source_t clk_src, clock_source_t sel)
{
	int32_t result = E_OK;

	uint32_t reg_addr = 0;

	switch (clk_src) {
		case SYSTEM_REF_CLOCK:
			reg_addr = REG_CLKGEN_SEL_400M;
			if (PLL0_CLOCK == sel) {
				raw_writel(reg_addr, 1);
			} else {
				raw_writel(reg_addr, 0);
			}
			break;

		case CPU_REF_CLOCK:
			reg_addr = REG_CLKGEN_SEL_300M;
			if (PLL1_CLOCK == sel) {
				raw_writel(reg_addr, 1);
			} else {
				raw_writel(reg_addr, 0);
			}
			break;
		default:
			result = E_PAR;
	}

	return result;
}

int32_t clock_divider(clock_source_t clk_src, uint32_t div)
{
	int32_t result = E_OK;

	if (div && !(div & 1)) {
		div -= 1;
	}

	switch (clk_src) {
		case APB_REF_CLOCK:
			raw_writel(REG_CLKGEN_DIV_APB_REF, div);
			break;
		case CAN0_CLOCK:
			raw_writel(REG_CLKGEN_DIV_CAN_0, div);
			break;
		case CAN1_CLOCK:
			raw_writel(REG_CLKGEN_DIV_CAN_1, div);
			break;
		case APB_CLOCK:
			raw_writel(REG_CLKGEN_DIV_APB, div);
			break;
		case AHB_CLOCK:
			raw_writel(REG_CLKGEN_DIV_AHB, div);
			break;
		case CPU_CLOCK:
			raw_writel(REG_CLKGEN_DIV_CPU, div);
			break;
		case MEM_CLOCK:
		case ROM_CLOCK:
		case RAM_CLOCK:
			raw_writel(REG_CLKGEN_DIV_MEM, div);
			break;
		default:
			result = E_PAR;
	}

	return result;
}

int32_t clock_frequency(clock_source_t clk_src)
{
	int32_t result = E_OK;

	uint32_t val = 0;

	switch (clk_src) {
		case SYSTEM_REF_CLOCK:
			if (raw_readl(REG_CLKGEN_SEL_400M)) {
				result = PLL0_OUTPUT_CLOCK_FREQ;
			} else {
				result = XTAL_CLOCK_FREQ;
			}
			break;
		case CPU_REF_CLOCK:
			if (raw_readl(REG_CLKGEN_SEL_300M)) {
				result = PLL1_OUTPUT_CLOCK_FREQ;
			} else {
				result = XTAL_CLOCK_FREQ;
			}
			break;
		case PLL0_CLOCK:
			result = PLL0_OUTPUT_CLOCK_FREQ;
			break;
		case PLL1_CLOCK:
			result = PLL1_OUTPUT_CLOCK_FREQ;
			break;

		case UART0_CLOCK:
		case UART1_CLOCK:
		case I2C_CLOCK:
		case SPI_M0_CLOCK:
		case SPI_M1_CLOCK:
		case SPI_S_CLOCK:
		case QSPI_CLOCK:
		case XIP_CLOCK:
		case APB_REF_CLOCK:
			val = raw_readl(REG_CLKGEN_DIV_APB_REF) & 0xF;
			result = clock_frequency(SYSTEM_REF_CLOCK);
			if (result > 0) {
				if (val) {
					result /= (val >> 1) << 1;
				}
			}
			break;

		//case GPIO_CLOCK:
		case TIMER_CLOCK:
		//case DMU_CLOCK:
		case PWM_CLOCK:
		case APB_CLOCK:
			val = raw_readl(REG_CLKGEN_DIV_AHB) & 0xF;
			result = clock_frequency(SYSTEM_REF_CLOCK);
			if (result > 0) {
#ifdef CHIP_ALPS_B
				if (val) {
					result /= (val + 1);
				}
#endif
				val = raw_readl(REG_CLKGEN_DIV_APB) & 0xF;
				if (val) {
					result /= (val + 1);
				}
			}
			break;

		case BB_CLOCK:
		case CRC_CLOCK:
		case AHB_CLOCK:
			val = raw_readl(REG_CLKGEN_DIV_AHB) & 0xF;
			result = clock_frequency(SYSTEM_REF_CLOCK);
			if (result > 0) {
				if (val) {
					result /= (val + 1);
				}
			}
			break;

		case CPU_CLOCK:
			val = raw_readl(REG_CLKGEN_DIV_CPU) & 0xF;
			result = clock_frequency(CPU_REF_CLOCK);
			if (result > 0) {
				if (val) {
					result /= (val + 1);
				}
			}
			break;
		/*
		case MEM_CLOCK:
		case ROM_CLOCK:
		case RAM_CLOCK:
			val = raw_readl(REG_CLKGEN_DIV_MEM) & 0xF;
			result = clock_frequency(CPU_WORK_CLOCK);
			if (result > 0) {
				if (val) {
					result /= (val + 1);
				}
			}
			break;
		*/
		case RC_BOOT_CLOCK:
			result = RC_BOOT_CLOCK_FREQ;
			break;
		case XTAL_CLOCK:
			result = XTAL_CLOCK_FREQ;
			break;
		case CAN0_CLOCK:
			val = raw_readl(REG_CLKGEN_DIV_CAN_0) & 0xF;
			result = clock_frequency(SYSTEM_REF_CLOCK);
			if (result > 0) {
				if (val) {
					result /= (val + 1);
				}
			}
			break;
		case CAN1_CLOCK:
			val = raw_readl(REG_CLKGEN_DIV_CAN_1) & 0xF;
			result = clock_frequency(SYSTEM_REF_CLOCK);
			if (result > 0) {
				if (val) {
					result /= (val + 1);
				}
			}
			break;
		case DAC_OUT_CLOCK:
		default:
			result = E_PAR;
	}

	return result;
}

void system_clock_init(void)
{
    /* CLKGEN_SEL_400M indicates whether refpll is locked in the last stage */
    /* CLKGEN_SEL_400M = 1, refpll locked */
    /* CLKGEN_SEL_400M = 0, refpll unlocked */
    int32_t result = E_OK;
    int32_t cpu_clk_ready = raw_readl(REG_CLKGEN_SEL_300M);

    if (!cpu_clk_ready) {
#ifndef PLAT_ENV_FPGA
        result = fmcw_radio_pll_clock_en();
#endif
    }

    /* If refpll is locked, set EMU_BOOT_DONE in 50M clk, */
    /* set clk div, enable 300M & 400M clk, do lock with temperature */
   if (E_OK == result) {
#ifdef CHIP_CASCADE
        fmcw_radio_clk_out_for_cascade();
#endif
/* CAN OTA will consume lots of time,so need to set boot done flag in order not to triggle func_safety */
/* firmware stage refer to: FUNC_SAFETY/SYSTEM_WATCHDOG/CPU_CLOCK_LOCK_DETECTOR */
/*    OTA   stage refer to: SYSTEM_CAN_OTA */
#if (defined(FUNC_SAFETY) || defined(SYSTEM_WATCHDOG) || defined(CPU_CLOCK_LOCK_DETECTOR) || defined(SYSTEM_CAN_OTA))
    if (0 == get_boot_state_flag()) {
        /* Swtich clock source to 50M clock */
        raw_writel(REG_CLKGEN_SEL_300M, 0);
        raw_writel(REG_CLKGEN_SEL_400M, 0);
        set_current_cpu_freq(XTAL_CLOCK_FREQ);

        /* EMU_BOOT_DONE (0xB00114) register should be configured when clock source 50M.  */
        /* In Boot state, write 1 to this register to transfer EMU_FSM to Operation state */
        /* This bit only need to be set when Functional-safety is enabled */
        set_boot_state_done();
    }
#endif
        raw_writel(REG_CLKGEN_DIV_AHB, 1);    // AHB should firstly be set.
        raw_writel(REG_CLKGEN_DIV_APB, 3);    // APB should be set after AHB
        raw_writel(REG_CLKGEN_DIV_CAN_0, 3);  // Set can_clk=400/4=100MHz
        raw_writel(REG_CLKGEN_DIV_CAN_1, 3);

        /* Swtich clock source to 300M & 400M clock */
        raw_writel(REG_CLKGEN_SEL_300M, 1);
        raw_writel(REG_CLKGEN_SEL_400M, 1);
        set_current_cpu_freq(PLL1_OUTPUT_CLOCK_FREQ);

#if REFPLL_CBANK == 1
        /* Auto-lock with temperature */
        result = fmcw_radio_pll_recal();
#endif
    }

    /* If refpll is unlocked after relock twice, do software reset */
    if(E_REFPLL_UNLOCK == result) {
        fmcw_radio_reset();
    }
}


void bus_clk_div(uint32_t div)
{
	clock_divider(AHB_CLOCK, div);
}

void apb_clk_div(uint32_t div)
{
	clock_divider(APB_CLOCK, div);
}

void apb_ref_clk_div(uint32_t div)
{
	clock_divider(APB_REF_CLOCK, div);
}

void switch_gpio_out_clk(bool ctrl)
{
	if (ctrl) {
            gpio_out_enable(1);
	} else {
            gpio_out_enable(0);
	}
}
