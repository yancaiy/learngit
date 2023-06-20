
@echo off
@rem "Clean the build obj of boot and firmware"

@rem Clean the firmware build obj
make clean
rd obj_alpsMP_ref_design_v1_sensor

@rem Delete firmware
rd /s /q build

@rem Clean the boot build obj
cd calterah/bootloader/baremetal/
make clean
rd obj_alpsMP_ref_design_v1_boot
