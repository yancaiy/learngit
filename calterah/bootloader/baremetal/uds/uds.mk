CAN_UDS_ROOT = $(CALTERAH_ROOT)/common/can_uds
CAN_UDS_BM_ROOT = ./uds

BOOT_INCDIRS += $(CAN_UDS_ROOT)

CAN_UDS_CSRCS += $(CAN_UDS_ROOT)/can_if.c
CAN_UDS_CSRCS += $(CAN_UDS_ROOT)/can_tp.c
CAN_UDS_CSRCS += $(CAN_UDS_ROOT)/uds_sd.c
CAN_UDS_CSRCS += $(CAN_UDS_ROOT)/init.c
CAN_UDS_CSRCS += $(CAN_UDS_ROOT)/uds.c
CAN_UDS_CSRCS += $(CAN_UDS_ROOT)/routine_control.c
CAN_UDS_CSRCS += $(CAN_UDS_ROOT)/routine_table.c

CAN_UDS_BM_CSRCS += uds/services.c
CAN_UDS_BM_CSRCS += uds/download.c
CAN_UDS_BM_CSRCS += uds/routine.c

CAN_UDS_COBJS = $(call get_calterah_objs, $(CAN_UDS_CSRCS))
CAN_UDS_BM_COBJS = $(call get_relobjs, $(CAN_UDS_BM_CSRCS))

CAN_UDS_BM_OUT_DIR = $(OUT_DIR)/uds
