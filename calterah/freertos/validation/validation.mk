# Application name
APPL ?= validation

# root dir of calterah
CALTERAH_ROOT ?= ../../

# Selected OS
OS_SEL ?= freertos

USE_BOARD_MAIN ?= 0

# root dir of embARC
EMBARC_ROOT ?= ../../../embarc_osp

CUR_CORE ?= arcem6

MID_SEL ?= common

BOARD ?= validation
BD_VER ?= 1

CHIP_CASCADE ?=

# application source dirs
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)
APPL_ASMSRC_DIR += $(APPLS_ROOT)/$(APPL)

ifeq ($(TOOLCHAIN), gnu)
	APPL_LIBS += -lm # math support
else

endif

SYSTEM_BOOT_STAGE ?= 2

FLASH_TYPE ?= s25fls

EXT_DEV_LIST ?= nor_flash

# To enable XIP function, "LOAD_XIP_TEXT_EN" in options/option.mk also need to be set to 1
FLASH_XIP ?= 0

UART_OTA ?= 1
SYSTEM_WATCHDOG ?=
SPIS_SERVER_ON ?=
FUNC_SAFETY_CONF ?=
CPU_CLOCK_LOCK_DETECTOR ?=
#  [NULL] -> dynamically obtain flash parameters; 1 -> use static flash parameters
FLASH_STATIC_PARAM ?=

#if PLL clock hasn't been opened before firmware, please set to a 1.
#it will indicate whether open PLL clock. and if PLL clock has been
#opened before, please set to a 0.
PLL_CLOCK_OPEN ?= 0

CAN_UDS ?= 1

# cpu usage statistic
CPU_STAT ?=

include $(CALTERAH_ROOT)/freertos/validation/config.mk

################# add new module at here! ###################
ifeq ($(TEST_UART_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/uart
endif
ifeq ($(TEST_GPIO_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/gpio
endif
ifeq ($(TEST_XIP_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/xip
endif
ifeq ($(TEST_DEBUG_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/debug_bb
endif
ifeq ($(TEST_SPI_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/spi
endif
ifeq ($(FLASH_XIP), 1)
ifeq ($(TEST_DMA_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/dma
endif
endif
ifeq ($(TEST_I2C_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/i2c
endif
ifeq ($(TEST_CAN_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/can
endif
ifeq ($(TEST_TIMER_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/timer
endif
ifeq ($(TEST_ROM_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/rom
endif
ifeq ($(TEST_DCORE_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/dcore
endif
ifeq ($(TEST_WDT_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/wdt
endif
ifeq ($(TEST_BB_EN), 1)
APPL_CSRC_DIR += $(APPLS_ROOT)/$(APPL)/cases/bb
endif
################ module validation end! #####################

# application include dirs
APPL_INC_DIR += $(APPLS_ROOT)/$(APPL) $(DEV_INCDIR) $(APPLS_ROOT)/$(APPL)/cases/debug_bb $(APPLS_ROOT)/$(APPL)/cases/bb

# include current project makefile
COMMON_COMPILE_PREREQUISITES += $(APPLS_ROOT)/$(APPL)/$(APPL).mk

### Options above must be added before include options.mk ###
# include key embARC build system makefile
override EMBARC_ROOT := $(strip $(subst \,/,$(EMBARC_ROOT)))
override CALTERAH_ROOT := $(strip $(subst \,/,$(CALTERAH_ROOT)))

include $(EMBARC_ROOT)/options/options.mk
EXTRA_DEFINES += -DACC_BB_BOOTUP=$(ACC_BB_BOOTUP)