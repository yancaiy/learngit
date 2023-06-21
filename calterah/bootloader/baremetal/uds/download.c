#include <string.h>
#include "embARC_toolchain.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "can_if.h"
#include "can_tp.h"
#include "uds_sd.h"
#include "services.h"
#include "init.h"


void can_post_init(void)
{
	/* TODO: */
}

void can_ota_main(void)
{
	/* init can link. */
	can_uds_init();

	arc_unlock_restore(0xff);

	/* client send DSC(Diagnostic session control) to switch session to extended session in UDS layer - 0x10. */

	/* post init. */
	//can_post_init();

	/* client send Control DTC Setting, off DTC in UDS layer - 0x85. */

	/* client send Communication Control, disable non-diagnostic communicationc - 0x28. */

	/* client read server identification. */

	/* client send link control to verify baud rate and switch baud rate. */

	/* client send DSC(Diagnostic session control) to switch session to programming session in UDS layer. */

	/* client send security access. */

	/* server write fingerprint. */

	/* if client request to update erase routine, then download erase routine firstly.
	 * and then client send routine Control to execute the routine. */

	/* client request to download image. */

	/* Client send Routine Control to check whether the program routine is successful. */

	/* Client send ECU reset. */

	while (1);
}
