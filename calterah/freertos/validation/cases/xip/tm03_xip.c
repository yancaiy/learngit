#include "tm03_xip.h"
#include "flash.h"
#include "flash_header.h"
#include "validation.h"
#include "vmodule.h"

static int32_t xip_read(xip_session_t *package);
static int32_t xip_erase(xip_session_t *package);
static int32_t xip_write(xip_session_t *package);


#define XIP_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_XIP_ID, cid, f_init, NULL)

int32_t xip_test(uint32_t id, xip_session_t *package)
{
	int32_t result = E_OK;

	switch (id) {
		case ID_1_READ:
			result = xip_read(package);
			break;
		case ID_2_ERASE:
			result = xip_erase(package);
			break;
		case ID_3_WRITE:
			result = xip_write(package);
			break;
		default:
			break;
	}

	return result;
}


static int32_t xip_read(xip_session_t *package)
{
	int32_t result = E_OK;

	if (package->type == eBYTE)
		result = flash_memory_readb(package->addr, (uint8_t *)package->rx_buf, package->len);
	else if(package->type == eWORD)
		result = flash_memory_readw(package->addr, package->rx_buf, package->len);
	else
		result = E_PAR;

	return result;
}


static int32_t xip_erase(xip_session_t *package)
{
	int32_t result = E_OK;

	result = flash_memory_erase(package->addr, package->len);

	return result;
}


static int32_t xip_write(xip_session_t *package)
{
	int32_t result = E_OK;

	if (package->type == eBYTE)
		result = flash_memory_writeb(package->addr, (uint8_t *)package->rx_buf, package->len);
	else if(package->type == eWORD)
		result = flash_memory_writew(package->addr, package->rx_buf, package->len);
	else
		result = E_PAR;

	return result;
}

/* get firmware version from flash header, then send it to PC by uart */
int32_t xip_get_fw_ver(void *self, void *params, uint32_t psize)
{
	int32_t result = E_OK;

	validation_case_t *case_ptr = (validation_case_t *)self;

	EMBARC_PRINTF("Module id:%d, Case id:%d \r\n", case_ptr->mod_id, case_ptr->case_id);

	/* TODO: need re-implemete this case */
	// flash_header_t *flash_header = NULL;

	// flash_header = flash_header_get();

	// EMBARC_PRINTF("Magic Number: %08X \n\r", flash_header->magic_number);

	return result;
}

XIP_VALIDATION_SYS_CASE(XIP_CID_1, xip_get_fw_ver);