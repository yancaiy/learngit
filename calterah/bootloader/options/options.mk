
CALTERAH_INC_DIR += $(CALTERAH_ROOT)/include
CALTERAH_INC_DIR += $(dir $(wildcard $(CALTERAH_ROOT)/common/*/))
CALTERAH_CSRC_DIR += $(dir $(wildcard $(CALTERAH_ROOT)/common/*/))
CALTERAH_CXXSRC_DIR += $(dir $(wildcard $(CALTERAH_ROOT)/common/*/))

## Include firmware version compilation file
include $(EMBARC_ROOT)/options/version.mk

ifneq ($(OS_SEL),)
COMMON_COMPILE_PREREQUISITES += $(CALTERAH_ROOT)/common/common.mk
COMMON_COMPILE_PREREQUISITES += $(CALTERAH_ROOT)/freertos/common/common.mk
include $(CALTERAH_ROOT)/common/common.mk
include $(CALTERAH_ROOT)/freertos/common/common.mk
endif
