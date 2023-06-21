#include "embARC.h"
#include "system.h"
#include "ota.h"
#include "flash.h"
#include "dw_uart.h"
#include "uart.h"
#include "dev_can.h"
#include "can_hal.h"
#include "init.h"
#include "fmcw_radio_reset.h"
#include "calterah_error.h"

#define REF_PLL_RELOCK_CNT (2)

int32_t fmcw_radio_pll_clock_en(void)
{
	bool lock_status = 0;
	int8_t relock_cnt = REF_PLL_RELOCK_CNT;

	/* In FLASH_BOOT_MODE, PLL parameters have been configured in ROMCODE.*/
	/* In other mode(HOST_BOOT_MODE or DEBUG_MODE), PLL paramters are not configured in ROMCODE. */
	/* For debugging purpose, add PLL parameters configuration in boot to enable 300M/400M clk.*/
	fmcw_radio_reg_write(NULL, 0x00, 0x00);
	fmcw_radio_reg_write(NULL, 0x03, 0x07);
	fmcw_radio_reg_write(NULL, 0x05, 0xc0);
	fmcw_radio_reg_write(NULL, 0x09, 0x20);
	fmcw_radio_reg_write(NULL, 0x12, 0xc0);
	fmcw_radio_reg_write(NULL, 0x13, 0xc0);
	fmcw_radio_reg_write(NULL, 0x14, 0xc0);
	fmcw_radio_reg_write(NULL, 0x15, 0xb0);
	fmcw_radio_reg_write(NULL, 0x16, 0xb0);
	fmcw_radio_reg_write(NULL, 0x1b, 0xeb);
	fmcw_radio_reg_write(NULL, 0x1d, 0x19);
	fmcw_radio_reg_write(NULL, 0x1e, 0xd0);
	fmcw_radio_reg_write(NULL, 0x25, 0xa0);
	fmcw_radio_reg_write(NULL, 0x26, 0x6f);

	/* Try refpll auto-lock twice */
	while (!lock_status && relock_cnt--) {
		fmcw_radio_reg_write(NULL, 0x00, 0x00);
		fmcw_radio_reg_write(NULL, 0x7d, 0x00);
		chip_hw_mdelay(2);
		fmcw_radio_reg_write(NULL, 0x7d, 0x01);
		chip_hw_mdelay(7);
		lock_status = fmcw_radio_reg_read_field(NULL, 0x7f, 0x00, 0x01);
	}

	if (lock_status) {
		return E_OK;
	} else {
		return E_REFPLL_UNLOCK;
	}
}

void fmcw_radio_clk_out_for_cascade(void)
{
}

void dummy_printf(const char*fmt, ...)
{
}

#ifdef __GNU__
void *memcpy(uint8_t *dst, uint8_t *src, uint32_t len)
{
	while (len--) {
		*dst++ = *src++;
	}
	return (void *)dst;
}
#endif
void init_hardware_hook(void)
{
	dcache_non_cached_region_base(REL_NON_CACHED_BASE);
}
#ifdef BOOT_SPLIT
void system_ota_entry(void)
#else
void board_main(void)
#endif
{
	int32_t result = 0;

	uint32_t reboot_mode = fmcw_radio_reboot_cause();

    /* Set cpu freq to 300M if CPU REF CLK is selected */
    if(raw_readl(REG_CLKGEN_SEL_300M)) {
        set_current_cpu_freq(PLL1_OUTPUT_CLOCK_FREQ);
    } else {
        set_current_cpu_freq(XTAL_CLOCK_FREQ);
    }

#ifndef BOOT_SPLIT
	system_clock_init();
	/*SW CRC is a must for OTA and Image Header CRC check*/
	gen_crc_table();
#endif

	timer_init();

	system_tick_init();

    /* By default, Boot use UART0(UART_OTA_ID ?= 0) for UART communication and OTA function */
    /* If you change to use UART1(UART_OTA_ID ?= 1), please call "io_mux_init()" here */
    /* The reason is that after Reset in "uart_ota" command, according to Alps Reference Manual,
       "DMU_MUX_UART_1" register change to 0x4 which means the function of "IO Mux for UART_1" change to GPIO,
       so need to call "io_mux_init" to recover the function of "IO Mux for UART_1" back to UART1 */
	uart_init();

	result = flash_init();
	if (0 != result) {
		reboot_mode = ECU_REBOOT_UART_OTA;
	}


	/* check the reboot flag... */
	switch (reboot_mode) {
		case ECU_REBOOT_UART_OTA:
			uart_ota_main();
			break;

		case ECU_REBOOT_CAN_OTA:
			io_mux_can0_func_sel(0);
			can_ota_main();
			break;
		default:
			if (E_FLASH_QUAD_ENTRY == normal_boot(1)) {
				/* Flash quad entry failed */
				fmcw_radio_reset();
			} else {
				/* Crc not matched or else */
				uart_ota_main();
			}
			break;
	}

	while (1);
}


void uart_print(uint32_t info)
{
#ifdef SYSTEM_UART_LOG_EN
	uint32_t len = 3 * sizeof(uint32_t);
	uint32_t msg[3] = {SYSTEM_LOG_START_FLAG, info, SYSTEM_LOG_END_FLAG};

	uint8_t *data_ptr = (uint8_t *)msg;
	uart_write(data_ptr, len);
#endif
}
