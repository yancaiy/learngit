
LIBC_LESS ?= 1
COMMON_STACK_INIT ?= 1

#SoC devices.
##############################
USE_CAN ?= 1
USE_UART ?= 1
USE_SPI ?=
USE_I2C ?=
USE_GPIO ?=
USE_OTP ?= 1

USE_DW_TIMER ?= 1
USE_SSI ?= 1
USE_QSPI ?=
USE_XIP ?= 0

USE_DMA ?=
BOOT_USE_HW_CRC ?= 1

#bootloader use enhance cache in arc_startup.s
BOOT_USE_ENHANCE_CACHE ?= 1

#BSP.
###############################
USE_NOR_FLASH ?= 1
FLASH_TYPE ?= s25fls
override FLASH_TYPE := $(strip $(FLASH_TYPE))

NOR_FLASH_STATIC_PARAM ?=

USE_PMIC ?= 1

#system component
###############################
USE_SYSTEM_TICK ?= 1

USE_FREEROTS_CLI ?=
USE_CAN_CLI ?=

CHIP_CASCADE ?=
CHIP_CASCADE_COST ?=

UART_LOG_EN ?= 1
UART_OTA ?= 1
UART_OTA_ID ?= 0

CAN_OTA ?= 0

#Application.
###############################
FMCW_SDM_FREQ ?= 400
CONSOLE_PRINTF ?= 

TICK_RATE_HZ ?=
TICK_MS_HZ ?= 1000

SYSTEM_BOOT_STAGE ?= 2

ELF_2_MULTI_BIN ?= 

#bootloader use XIP mode boot.
###############################
FLASH_XIP ?= 0

ifeq ($(FLASH_XIP), 1)
	USE_XIP = 1
endif