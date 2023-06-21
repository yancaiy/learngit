/*
 * Copyright (c) 2020, 加特兰微电子科技（上海）有限公司. All rights reserved.
 * Function : ecc test case
 * Date     : 2020年12月01日
 */

#include <stdlib.h>
#include "embARC.h"
#include "embARC_debug.h"
#include "embARC_assert.h"

#include "validation.h"
#include "debug_hal.h"
#include "debug.h"
#include "sensor_config.h"
#include "tm12_ecc.h"

#define ECC_VALIDATION_SYS_CASE(cid, f_init)  \
	VALIDATION_SYS_CASE_DEFINE(MODULE_ECC_ID, cid, f_init, NULL)

#define ECC_VALIDATION_CASE(cid, f_entry, f_exit) \
	VALIDATION_CASE_DEFINE(ecc, MODULE_ECC_ID, cid, f_entry, f_exit)


uint32_t g_isr_cnt = 0;
uint32_t g_test_delay = 0;
uint32_t g_ecc_value = 0;

void clear_ecc(void)
{
	// SYSTEM_SRAM_ECC_DB_ERR_CLR
	raw_writel(0xa0000310, 1);
	// SYSTEM_SRAM_ECC_SB_ERR_CLR
	raw_writel(0xa000030c, 1);

	// SYSTEM_SRAM_ECC_SB_ERR_CLR
	raw_writel(0xa000020c, 1);
	// SYSTEM_ROM_ECC_DB_ERR_CLR
	raw_writel(0xa0000210, 1);
	// SAFETY_ECC_ERR_STA
	EMBARC_PRINTF("clear ecc. ecc sta[%x]\r\n", raw_readl(0xa0300404)); 
}


static void ecc_isr(void *param)
{
	g_isr_cnt++;
	raw_writel(0xa1500000, 0x64);
	// SYSTEM_SRAM_ECC_SB_ERR_CLR
	// raw_writel(0xa000030c, 1);
	// SYSTEM_SRAM_ECC_DB_ERR_CLR
	// EMBARC_PRINTF(" ecc sta sta[%x]\r\n", raw_readl(0xa0300404));
	// raw_writel(0xa0000310, 1);
    // dccm test

	EMBARC_PRINTF("clear\r\n");
}


void ecc_intr_register(void) 
{
	uint32_t result = int_handler_install(INT_DG_ERROR_IRQ, ecc_isr);
	
	if (result != E_OK) {
		EMBARC_PRINTF("intr ecc install fail\r\n");
		return;
	} 

	int_enable(INT_DG_ERROR_IRQ);
	
	EMBARC_PRINTF("ecc install success.\r\n");
	return;
}

static int32_t ecc_case_init(void *self, void *params, uint32_t len)
{
	/* ECC 校验失败中断上报 */
	ecc_intr_register();
}

static int32_t ecc_case_exit(void *self)
{
	return E_OK;
}

static int32_t ecc_sram_single_case_entry(void *self, void *params, uint32_t len)
{
	static unsigned int test_done = 0;
	if (g_test_delay < 5) {
		return;
	} else {
		EMBARC_PRINTF("[main] ecc test start \r\n");	
	}
	if (test_done) {
		EMBARC_PRINTF("[main] ecc read1:%x\r\n", raw_readl(0xa0300404));
		return;
	}
	/* 1 cpu enable intr 22 *//* 2. 3. enable clock */
	raw_writel(0xa0100424, 1);
	raw_writel(0xa0100420, 1);
	/* 3 ecc isr enable */
	raw_writel(0xa0300400, 0x4);

	/* 4 SYSTEM_SRAM_ECC_EN 1 */
	raw_writel(0xa0000300, 1);
	
	*(uint32_t *)0x5003e000 = 0x100;

	//EMBARC_PRINTF("[main] first W:%x\r\n", *(uint32_t *)0x50000000);
	
	raw_writel(0xa0000300, 0);
	*(uint32_t *)0x5003e000 = 0x101;

	raw_writel(0xa0000300, 1);
	g_ecc_value = *(uint32_t *)0x5003e000;

	// expected trigger an irq[INT_RF_ERROR_SS1]
	//EMBARC_PRINTF("[main] trigger read:%x\r\n", *(uint32_t *)0x5003e000);
	//chip_hw_mdelay(500);
	//EMBARC_PRINTF("[main] ecc read0:%x\r\n", raw_readl(0xa0300404));
	raw_writel(0xa0000300, 0);
	//raw_writel(0xa000030c, 1);
	test_done = 1;

}

static int32_t ecc_sram_double_case_entry(void *self, void *params, uint32_t len)
{
	static unsigned int test_done = 0;
	if (g_test_delay < 5) {
		return;
	} else {
		EMBARC_PRINTF("[main] ecc test start \r\n");	
	}
	if (test_done) {
		EMBARC_PRINTF("[main] ecc read1:%x\r\n", raw_readl(0xa0300404));
		return;
	}
	/* 1 cpu enable intr 22 *//* 2. 3. enable clock */
	raw_writel(0xa0100424, 1);
	raw_writel(0xa0100420, 1);
	/* 3 ecc isr enable */
	raw_writel(0xa0300400, 0x8);

	/* 4 SYSTEM_SRAM_ECC_EN 1 */
	raw_writel(0xa0000300, 1);
	
	*(uint32_t *)0x5003e000 = 0x100;

	//EMBARC_PRINTF("[main] first W:%x\r\n", *(uint32_t *)0x50000000);
	
	raw_writel(0xa0000300, 0);
	*(uint32_t *)0x5003e000 = 0x103;

	raw_writel(0xa0000300, 1);
	g_ecc_value = *(uint32_t *)0x5003e000;

	// expected trigger an irq[INT_RF_ERROR_SS1]
	//EMBARC_PRINTF("[main] trigger read:%x\r\n", *(uint32_t *)0x5003e000);
	//chip_hw_mdelay(500);
	//EMBARC_PRINTF("[main] ecc read0:%x\r\n", raw_readl(0xa0300404));
	raw_writel(0xa0000300, 0);
	//raw_writel(0xa000030c, 1);
	test_done = 1;

}


static int32_t ecc_dccm_single_bit_case_entry(void *self, void *params, uint32_t len)
{
	static unsigned int test_done = 0;
    volatile uint32_t  *addr = (uint32_t *)0x80000000;


	if (test_delay < 5) {
		return;
	} else {
		EMBARC_PRINTF("[main] ecc test start \r\n");	
	}
	if (test_done) {
		EMBARC_PRINTF("[main] ecc read1:%x\r\n", 
			raw_readl(0xa0300404));
		return;
	}
	raw_writel(0xa0100424, 1);
	raw_writel(0xa0100420, 1);
	EMBARC_PRINTF("[main] intrst st0 :%x aux_0x3f[%x]\r\n", 
		raw_readl(0xa0300304), _arc_aux_read(0x3f));


	// SAFETY_CPU_ERR_IRQ_ENA  bit0 sb/bit1 db,  ecc: CPU Test Interrupt Enable
	raw_writel(0xa0300300, 1); 
	
	*addr = 0x12345678;
	
	/* trig */
	_arc_aux_write(0x3f, 0xe);

	
	*addr = 0x12345670;
	EMBARC_PRINTF("[main] intrst *addr[%x]\r\n", 
		*addr);
	
	_arc_aux_write(0x3f, 0xf);
	resultg = *addr;
	EMBARC_PRINTF("[main] intrst st1 :%x aux_0x3f[%x]\r\n", 
		raw_readl(0xa0300304), _arc_aux_read(0x3f));
	EMBARC_PRINTF("[main] intrst st2 :%x aux_0x3f[%x]\r\n", 
		raw_readl(0xa0300304), _arc_aux_read(0x3f));
	
	//raw_writel(0xa0300538, 0);
	//raw_writel(0xa000030c, 1);
	test_done = 1;

}


static int32_t ecc_dccm_double_bit_case_entry(void *self, void *params, uint32_t len)
{
	static unsigned int test_done = 0;
    uint32_t        *addr = (uint32_t *)0x80000000;

	if (g_test_delay < 5) {
		return;
	} else {
		EMBARC_PRINTF("[main] ecc test start \r\n");	
	}
	if (test_done) {
		EMBARC_PRINTF("[main] ecc read1:%x\r\n", raw_readl(0xa0300404));
		return;
	}
	raw_writel(0xa0100424, 1);
	raw_writel(0xa0300308, 1);
	// SAFETY_CPU_ERR_IRQ_ENA  bit0 sb/bit1 db,  ecc: CPU Test Interrupt Enable
	raw_writel(0xa0300300, 0x2); 
	
	*addr = 0x12345678;
	
	/* trig */
	_arc_aux_write(0x3f, 0xe);
	//  0x12345678  -> 0x1234567b
	*addr = 0x1234567b; 
	_arc_aux_write(0x3f, 0xf);
	g_ecc_value = *(uint32_t *)*addr;
	EMBARC_PRINTF("[main] first W:%x\r\n", g_ecc_value);
	

	test_done = 1;

}


static int32_t  ecc_bbsram_single_bit_case_entry(void *self, void *params, uint32_t len)
{
	static uint32_t test_done = 0;
	uint32_t        g_test_delay = *(uint32_t *)params;
	int32_t         result = E_OK;
	uint32_t        *addr = (uint32_t *)0x80000000;

	if (params == NULL) {
		return E_PAR;
	}
	
	if (g_test_delay < 5) {
		return E_OK;
	}
	if (test_done) {
		EMBARC_PRINTF("[main] ecc read1:%x\r\n", raw_readl(0xa0300404));
		return E_OK;
	}
	/*
		26.4.3 CPU Tests
		26.4.3.1 Core 0 DCCM Single Bit ECC Error
		Safety SS1 Error
		1. set CLOCK_ENA_SAFETY to 0x1   
		2. set SAFETY_CPU_ERR_SS1_ENA[0] to 0x1
		3. set SAFETY_SS1_CTRL to 0x1
		4. wait to enter SS1 state, auto force reset & reboot
		5. check SAFETY_ECC_ERR_SS1_CODE_0[0]
	*/
	
	/* 1 cpu enable intr 22 *//* 2. 3. enable clock */
	raw_writel(0xa0100424, 1);
	raw_writel(0xa0300308, 1);
	
	raw_writel(0xa0300014, 1);

	*addr = 0x12345678;

	/* trig */
    _arc_aux_write(0x3f, 0xe);
	*addr = 0x12345677;
    _arc_aux_write(0x3f, 0xf);

	EMBARC_PRINTF("[main] first W:%x\r\n", *(uint32_t *)*addr);
	
	raw_writel(0xa0300538, 0);
	test_done = 1;
	return E_OK;
}

static int32_t  ecc_bbsram_double_bit_case_entry(void *self, void *params, uint32_t len)
{
	static uint32_t test_done = 0;
	uint32_t        g_test_delay = *(uint32_t *)params;
	int32_t         result = E_OK;
	uint32_t        *addr = (uint32_t *)0x80000000;

	if (params == NULL) {
		return E_PAR;
	}
	
	if (g_test_delay < 5) {
		return E_OK;
	}
	if (test_done) {
		EMBARC_PRINTF("[main] ecc read1:%x\r\n", raw_readl(0xa0300404));
		return E_OK;
	}
	/*
		26.4.3 CPU Tests
		26.4.3.1 Core 0 DCCM Single Bit ECC Error
		Safety SS1 Error
		1. set CLOCK_ENA_SAFETY to 0x1   
		2. set SAFETY_CPU_ERR_SS1_ENA[0] to 0x1
		3. set SAFETY_SS1_CTRL to 0x1
		4. wait to enter SS1 state, auto force reset & reboot
		5. check SAFETY_ECC_ERR_SS1_CODE_0[0]
	*/
	
	/* 1 cpu enable intr 22 *//* 2. 3. enable clock */
	raw_writel(0xa0100424, 1);
	raw_writel(0xa0300308, 1);
	
	raw_writel(0xa0300014, 1);

	*addr = 0x12345678;

	/* trig */
    _arc_aux_write(0x3f, 0xe);
	*addr = 0x12345677;
    _arc_aux_write(0x3f, 0xf);

	EMBARC_PRINTF("[main] first W:%x\r\n", *(uint32_t *)*addr);
	
	raw_writel(0xa0300538, 0);
	test_done = 1;
	return E_OK;
}


ECC_VALIDATION_SYS_CASE(ECC_CID_0, ecc_case_init);
ECC_VALIDATION_CASE(ECC_CID_0, ecc_sram_single_case_entry, ecc_case_exit);

ECC_VALIDATION_SYS_CASE(ECC_CID_1, ecc_sram_double_case);
ECC_VALIDATION_CASE(ECC_CID_1, ecc_sram_double_case_entry, ecc_case_exit);

ECC_VALIDATION_SYS_CASE(ECC_CID_2, ecc_sram_double_case);
ECC_VALIDATION_CASE(ECC_CID_2, ecc_dccm_single_bit_case_entry, ecc_case_exit);


ECC_VALIDATION_SYS_CASE(ECC_CID_3, ecc_sram_double_case);
ECC_VALIDATION_CASE(ECC_CID_3, ecc_dccm_double_bit_case_entry, ecc_case_exit);


ECC_VALIDATION_SYS_CASE(ECC_CID_4, ecc_case_init);
ECC_VALIDATION_CASE(ECC_CID_4, ecc_bbsram_single_case_entry, ecc_case_exit);

ECC_VALIDATION_SYS_CASE(ECC_CID_5, ecc_case_init);
ECC_VALIDATION_CASE(ECC_CID_5, ecc_bbsram_double_case_entry, ecc_case_exit);

