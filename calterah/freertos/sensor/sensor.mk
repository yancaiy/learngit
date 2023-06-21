# Application name
APPL ?= sensor

# root dir of calterah
CALTERAH_ROOT ?= ../../

# Selected OS
OS_SEL ?= freertos

USE_BOARD_MAIN ?= 0

# root dir of embARC
EMBARC_ROOT ?= ../../../embarc_osp


CUR_CORE ?= arcem6

MID_SEL ?= common

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
CPU_CLOCK_LOCK_DETECTOR ?=

# function safety config 
# MUST set FUNC_SAFETY_CONF=1 if open function safety
# then you must config SM_xxx item ON/OFF，if not,it's default off
FUNC_SAFETY_CONF ?=
FUNC_SAFETY_CLI ?=1

SAFETY_FEATURE_SM1 ?=
SAFETY_FEATURE_SM2 ?=
SAFETY_FEATURE_SM3 ?=
SAFETY_FEATURE_SM4 ?=
SAFETY_FEATURE_SM5 ?=
SAFETY_FEATURE_SM6 ?=
SAFETY_FEATURE_SM8 ?=
SAFETY_FEATURE_SM11 ?=
SAFETY_FEATURE_SM12 ?=
SAFETY_FEATURE_SM13 ?=
SAFETY_FEATURE_SM14 ?=
SAFETY_FEATURE_SM101 ?=
SAFETY_FEATURE_SM102 ?=
SAFETY_FEATURE_SM103 ?=
SAFETY_FEATURE_SM104 ?=
SAFETY_FEATURE_SM105 ?=
SAFETY_FEATURE_SM106 ?=
SAFETY_FEATURE_SM107 ?=
SAFETY_FEATURE_SM108 ?=
SAFETY_FEATURE_SM109 ?=
#SAFETY_FEATURE_SM120 ?=
SAFETY_FEATURE_SM121 ?=
#SAFETY_FEATURE_SM122 ?=
#SAFETY_FEATURE_SM123 ?=
#SAFETY_FEATURE_SM124 ?=
#SAFETY_FEATURE_SM125 ?=
SAFETY_FEATURE_SM126 ?=
#SAFETY_FEATURE_SM129 ?=
SAFETY_FEATURE_SM130 ?=
SAFETY_FEATURE_SM133 ?=
SAFETY_FEATURE_SM201 ?=
#SAFETY_FEATURE_SM202 ?=
#SAFETY_FEATURE_SM206 ?=
#SAFETY_FEATURE_SM805 ?=
SAFETY_FEATURE_SM901 ?=
SAFETY_FEATURE_SM902 ?=
SAFETY_FEATURE_SM904 ?=
SAFETY_FEATURE_SM905 ?=
SAFETY_FEATURE_SM906 ?=
SAFETY_FEATURE_SM907 ?=
SAFETY_FEATURE_SM908 ?=
SAFETY_FEATURE_SM910 ?=
SAFETY_FEATURE_SM911 ?=

#  [NULL] -> dynamically obtain flash parameters; 1 -> use static flash parameters
FLASH_STATIC_PARAM ?=

#if PLL clock hasn't been opened before firmware, please set to a 1.
#it will indicate whether open PLL clock. and if PLL clock has been
#opened before, please set to a 0.
PLL_CLOCK_OPEN ?= 0

CAN_UDS ?= 1

# cpu usage statistic
CPU_STAT ?=

# application include dirs
APPL_INC_DIR += $(APPLS_ROOT)/$(APPL)

# include current project makefile
COMMON_COMPILE_PREREQUISITES += $(APPLS_ROOT)/$(APPL)/$(APPL).mk

### Options above must be added before include options.mk ###
# include key embARC build system makefile
override EMBARC_ROOT := $(strip $(subst \,/,$(EMBARC_ROOT)))
override CALTERAH_ROOT := $(strip $(subst \,/,$(CALTERAH_ROOT)))
include $(EMBARC_ROOT)/options/options.mk
EXTRA_DEFINES += -DACC_BB_BOOTUP=$(ACC_BB_BOOTUP)