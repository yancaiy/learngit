CALTERAH_COMMON_ROOT = $(CALTERAH_ROOT)/common


include $(CALTERAH_COMMON_ROOT)/math/math.mk
include $(CALTERAH_COMMON_ROOT)/fmcw_radio/fmcw_radio.mk
include $(CALTERAH_COMMON_ROOT)/baseband/baseband.mk
include $(CALTERAH_COMMON_ROOT)/sensor_config/sensor_config.mk
include $(CALTERAH_COMMON_ROOT)/track/track.mk
#include $(CALTERAH_COMMON_ROOT)/track/track_release.mk
include $(CALTERAH_COMMON_ROOT)/vel_deamb_MF/vel_deamb_MF.mk
include $(CALTERAH_COMMON_ROOT)/bb_flow/bb_flow.mk
include $(CALTERAH_COMMON_ROOT)/cluster/cluster.mk
include $(CALTERAH_COMMON_ROOT)/emu/emu.mk
include $(CALTERAH_COMMON_ROOT)/can_cli/can_cli.mk

ifneq ($(CAN_UDS), )
include $(CALTERAH_COMMON_ROOT)/can_uds/can_uds.mk
endif

include $(CALTERAH_COMMON_ROOT)/console/console.mk

ifneq ($(CHIP_CASCADE),)
include $(CALTERAH_COMMON_ROOT)/cascade/cascade.mk
endif

ifneq ($(SPIS_SERVER_ON),)
include $(CALTERAH_COMMON_ROOT)/spi_slave_server/spi_slave_server.mk
endif

ifneq ($(FUNC_SAFETY_CONF),)
include $(CALTERAH_COMMON_ROOT)/fsm/fsm.mk
endif

include $(CALTERAH_COMMON_ROOT)/spi_master/spi_master.mk

ifneq ($(CPU_STAT),)
include $(CALTERAH_COMMON_ROOT)/statistic/statistic.mk
endif

# List of Features
FMCW_SDM_FREQ ?= 400
INTER_FRAME_POWER_SAVE ?= 1
TRK_CONF_3D ?= 0
MAX_OUTPUT_OBJS ?= 64
NUM_VARRAY ?= 1
NUM_FRAME_TYPE ?= 1
BB_INTERFERENCE_CHECK ?= 0
FRAME_CYCLE_TIME_CHECK ?= 0

# Accelerating bootsup speed
#    0: no acceleration;
#    1: generating accelerating code;
#    2: using generated code to speedup bootsup;
ACC_BB_BOOTUP ?= 0

ifeq ($(CHIP_VER),B)
CHK_DMU_MODE ?= 1
else
CHK_DMU_MODE ?= 0
endif
HTOL_TEST ?= 0
DOPPLER_MOVE ?= 1
AUTO_TX ?= 0
REFPLL_CBANK ?= 1
RF_COMP ?= 1

ifneq ($(CAN_UDS),)
EXTRA_DEFINES += -DCAN_UDS
endif

EXTRA_DEFINES += -DFMCW_SDM_FREQ=$(FMCW_SDM_FREQ)
EXTRA_DEFINES += -DINTER_FRAME_POWER_SAVE=$(INTER_FRAME_POWER_SAVE)
EXTRA_DEFINES += -DTRK_CONF_3D=$(TRK_CONF_3D)
EXTRA_DEFINES += -DMAX_OUTPUT_OBJS=$(MAX_OUTPUT_OBJS)
EXTRA_DEFINES += -DNUM_VARRAY=$(NUM_VARRAY)
EXTRA_DEFINES += -DNUM_FRAME_TYPE=$(NUM_FRAME_TYPE)
EXTRA_DEFINES += -DCHK_DMU_MODE=$(CHK_DMU_MODE)
EXTRA_DEFINES += -DHTOL_TEST=$(HTOL_TEST)
EXTRA_DEFINES += -DACC_BB_BOOTUP=$(ACC_BB_BOOTUP)
EXTRA_DEFINES += -DDOPPLER_MOVE=$(DOPPLER_MOVE)
EXTRA_DEFINES += -DAUTO_TX=$(AUTO_TX)
EXTRA_DEFINES += -DREFPLL_CBANK=$(REFPLL_CBANK)
EXTRA_DEFINES += -DRF_COMP=$(RF_COMP)
EXTRA_DEFINES += -DBB_INTERFERENCE_CHECK=$(BB_INTERFERENCE_CHECK)
EXTRA_DEFINES += -DFRAME_CYCLE_TIME_CHECK=$(FRAME_CYCLE_TIME_CHECK)

