#include "tm11_dcore.h"

#include "validation.h"
#include "vmodule.h"

#define DCORE_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_DCORE_ID, cid, f_init, NULL)

/* Init Core 1 */
int32_t dcore_module_init(void *self, void *params, uint32_t psize)
{

}

DCORE_VALIDATION_SYS_CASE(DCORE_CID_0, dcore_module_init);
