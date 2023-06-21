import os,argparse,sys,subprocess
from os import path

def check_bootup_file(src_dir):
    tar_file = src_dir + "\\calterah\\common\\baseband\\baseband_bb_bootup.h"
    if os.path.exists(tar_file):
        return True
    else:
        return False

def cmd_subprocess(src_dir, cmd):
    process = subprocess.Popen(cmd, cwd=src_dir, shell=True)
    return process

def main(args):
    vendor_list = ["s25fls","giga","winbond", "micron", "microchip", "mxic", "issi"]
    fw_cwd = os.getcwd()
    bt_cwd = fw_cwd + "\\calterah\\bootloader\\"
    cmb_cmd = "python combine.py -appl {0} -tf {1} -elf {2} -board {3} -tc {4} -v {5} -aes {6} -stage {7} -method {8} -bmethod {9}".format(args.appl,args.timefolder,args.elfdump,args.board,args.toolchain,args.vendor,args.aes,args.stage,args.method,args.bmethod)

    # Toolchain metaware
    if args.toolchain.lower() == "mw":
        log = "Toolchain: mw\r\n"
        # Cascade
        if args.cascade.lower() == "true":
            bt_cmd = "make clean TOOLCHAIN=mw && make bin TOOLCHAIN=mw "
            fw_cmd = "make clean TOOLCHAIN=mw BOARD=ref_cascade && make bin TOOLCHAIN=mw BOARD=ref_cascade "
            gen_header_fw_cmd = "python post.py -fbin obj_alpsMP_ref_cascade_v1_sensor/mw_arcem6/sensor_mw_arcem6.bin -obin firmware.bin "
            gen_header_bt_cmd = "python post.py -bbin calterah/baremetal/obj_alpsMP_ref_design_v1_boot/mw_arcem6/boot_mw_arcem6.bin -obin boot.bin -fmethod 0 "
            log += "Board: ref_cascade\r\n"
        # Non-cascade
        else:
            bt_cmd = "make clean TOOLCHAIN=mw && make bin TOOLCHAIN=mw "
            fw_cmd = "make clean TOOLCHAIN=mw BOARD={0} APPL={1} && make bin TOOLCHAIN=mw BOARD={0} APPL={1}".format(args.board,args.appl)
            gen_header_fw_cmd = "python post.py -fbin obj_alpsMP_{0}_v1_{1}/mw_arcem6/{1}_mw_arcem6.bin -obin firmware.bin ".format(args.board,args.appl)
            gen_header_bt_cmd = "python post.py -bbin calterah/baremetal/obj_alpsMP_ref_design_v1_boot/mw_arcem6/boot_mw_arcem6.bin -obin boot.bin -fmethod 0 "
            log += "Board: ref_design\r\n"
    # Toolchain gnu
    elif args.toolchain.lower() == "gnu":
        log = "Toolchain: gnu\r\n"
        # Cascade
        if args.cascade.lower() == "true":
            bt_cmd = "make clean && make bin "
            fw_cmd = "make clean BOARD=ref_cascade && make bin BOARD=ref_cascade "
            gen_header_fw_cmd = "python post.py -fbin obj_alpsMP_ref_cascade_v1_sensor/gnu_arcem6/sensor_gnu_arcem6.bin -obin firmware.bin "
            gen_header_bt_cmd = "python post.py -bbin calterah/baremetal/obj_alpsMP_ref_design_v1_boot/gnu_arcem6/boot_gnu_arcem6.bin -obin boot.bin -fmethod 0 "
            log += "Board: ref_cascade\r\n"
        # Non-cascade
        else:
            bt_cmd = "make clean && make bin "
            fw_cmd = "make clean BOARD={0} APPL={1} && make bin BOARD={0} APPL={1} ".format(args.board,args.appl)
            gen_header_fw_cmd = "python post.py -fbin obj_alpsMP_{0}_v1_{1}/gnu_arcem6/{1}_gnu_arcem6.bin -obin firmware.bin ".format(args.board,args.appl)
            gen_header_bt_cmd = "python post.py -bbin calterah/baremetal/obj_alpsMP_ref_design_v1_boot/gnu_arcem6/boot_gnu_arcem6.bin -obin boot.bin -fmethod 0 "
            log += "Board: ref_design\r\n"
    else:
        raise Exception("Invalid toolchain input")

    # Warning 2 Error build option
    setW2E = " WARNING_TO_ERROR=0 "
    if args.warning2error == "true":
        setW2E = " WARNING_TO_ERROR=1 "

    bt_cmd += setW2E
    fw_cmd += setW2E

    if args.macro != None:
        fw_cmd += ' ' .join(args.macro)
        fw_cmd += " "
    else:
        print("\t\t      : args.macro null")

    # 2 stage
    if args.stage == "2":
        gen_header_bt_cmd = ""
        bt_cmd = ""
        fw_cmd += "SYSTEM_BOOT_STAGE=2 "

        # 2 stage xip
        if args.method.lower() == "xip":
            gen_header_fw_cmd += "-stage 2 "
            gen_header_fw_cmd += "-bsplit 0 "
            gen_header_fw_cmd += "-fmethod 1 "
            fw_cmd += "FLASH_XIP=1 LOAD_XIP_TEXT_EN=1 "
            log += "Mode: 2 stage xip\r\n"

        # 2 stage ram
        elif args.method.lower() == "ram":
            # 2 stage ram
            gen_header_fw_cmd += "-stage 2 "
            gen_header_fw_cmd += "-bsplit 0 "
            gen_header_fw_cmd += "-fmethod 0 "
            log += "Mode: 2 stage ram\r\n"
        else:
            raise Exception("Invalid method input")

        # Enable AES in post.py
        if args.aes.lower() == "true":
            gen_header_fw_cmd += "-aes 1 "
            log += "aes=enable "
        else:
            gen_header_fw_cmd += "-aes 0 "
            log += "ase=disable "

        # Flash vendor
        if args.vendor in vendor_list:
            fw_cmd += "FLASH_TYPE=%s " % args.vendor
            log += "Option: flash=%s " % args.vendor
        else:
            raise Exception("Invalid flash vendor input")
    # 3 stage
    elif args.stage == "3":
        bt_cmd += "SYSTEM_BOOT_STAGE=3 "
        fw_cmd += "SYSTEM_BOOT_STAGE=3 "

        # 3 stage split xip
        if args.split.lower() == "true":
            # 3 stage split xip
            gen_header_fw_cmd += "-stage 3 "
            gen_header_fw_cmd += "-bsplit 1 "
            gen_header_fw_cmd += "-fmethod 1 "

            gen_header_bt_cmd += "-stage 3 "
            gen_header_bt_cmd += "-bsplit 1 "
            fw_cmd += "FLASH_XIP=1 LOAD_XIP_TEXT_EN=1 "
            bt_cmd += "ELF_2_MULTI_BIN=1 "
            log += "Mode: 3 stage boot-split xip\r\n"

        # 3 stage non-split
        else:
            gen_header_fw_cmd += "-stage 3 "
            gen_header_fw_cmd += "-bsplit 0 "

            gen_header_bt_cmd += "-stage 3 "
            gen_header_bt_cmd += "-bsplit 0 "

            # 3 stage non-split bootloader xip
            if args.bmethod.lower() == "xip":
                bt_cmd += "FLASH_XIP=1 LOAD_XIP_TEXT_EN=1 "
                fw_cmd += " BOOTLOADER_FLASH_XIP=1 "
                gen_header_fw_cmd += "-bmethod 1 "
                gen_header_bt_cmd += "-bmethod 1 "
            else:
                gen_header_fw_cmd += "-bmethod 0 "
                gen_header_bt_cmd += "-bmethod 0 "

            # 3 stage non-split xip
            if args.method.lower() == "xip":
                fw_cmd += "FLASH_XIP=1 LOAD_XIP_TEXT_EN=1 "
                gen_header_fw_cmd += "-fmethod 1 "
                log += "Mode: 3 stage xip\r\n"

            # 3 stage non-split ram
            else:
                gen_header_fw_cmd += "-fmethod 0 "
                log += "Mode: 3 stage ram\r\n"
            
            if args.hwcrc.lower() == "1":
                gen_header_fw_cmd += "-hwcrc 1 "
                gen_header_bt_cmd += "-hwcrc 1 "

                bt_cmd += "BOOT_USE_HW_CRC=1 "
            else:
                gen_header_fw_cmd += "-hwcrc 0 "
                gen_header_bt_cmd += "-hwcrc 0 "
                bt_cmd += "BOOT_USE_HW_CRC= "

            if args.xipload.lower() == "true":
                gen_header_fw_cmd += "-flxip 1 "
                gen_header_bt_cmd += "-flxip 1 "
                bt_cmd += " BOOT_USE_XIP_LOAD_FW=1 "
            else:
                gen_header_fw_cmd += "-flxip 0 "
                gen_header_bt_cmd += "-flxip 0 "
                bt_cmd += " BOOT_USE_XIP_LOAD_FW=0 "

        # Enable AES in post.py
        if args.aes.lower() == "true":
            gen_header_fw_cmd += "-aes 1 "
            gen_header_bt_cmd += "-aes 1 "
            log += "aes=enable "
        else:
            gen_header_fw_cmd += "-aes 0 "
            gen_header_bt_cmd += "-aes 0 "
            log += "ase=disable "

        # Flash vendor
        if args.vendor in vendor_list:
            fw_cmd += "FLASH_TYPE=%s " % args.vendor
            bt_cmd += "FLASH_TYPE=%s " % args.vendor
            log += "Option: flash=%s " % args.vendor
        else:
            raise Exception("Invalid flash vendor input")
    else:
        raise Exception("Invalid stage input")

    # Set firmware version
    if args.firmware.lower() == "release":
        fw_cmd += "FW=release "
        log += "fw=release "
    elif args.firmware.lower() == "debug":
        fw_cmd += "FW=debug "
        log += "fw=debug "
    else:
        raise Exception("Invalid FW input")

    # Set multi-thread compiling
    if args.thread.isnumeric():
        fw_cmd += "-j %d " % int(args.thread)
    else:
        raise Exception("Invalid thread input")

    # Enable baseband acceleration
    if args.bbacc.lower() == "true":
        if check_bootup_file(fw_cwd):
            fw_cmd += "ACC_BB_BOOTUP=2 "
            log += "bb_acc=enable"
        else:
            raise Exception("baseband_bb_bootup.h not found")
    else:
        log += "bb_acc=disable"

    # Enable HAL test cases, only if "-board=validation, -appl=validation"
    if args.board == "validation":
        if args.test_uart == "1":
            fw_cmd += " TEST_UART_EN={0} ".format(args.test_uart)
        if args.test_gpio == "1":
            fw_cmd += "TEST_GPIO_EN={0} ".format(args.test_gpio)
        if args.test_xip == "1":
            fw_cmd += "TEST_XIP_EN={0} ".format(args.test_xip)
        if args.test_debug == "1":
            fw_cmd += "TEST_DEBUG_EN={0} ".format(args.test_debug)
        if args.test_spi == "1":
            fw_cmd += "TEST_SPI_EN={0} ".format(args.test_spi)
        if args.test_dma == "1":
            fw_cmd += "TEST_DMA_EN={0} ".format(args.test_dma)
        if args.test_i2c == "1":
            fw_cmd += "TEST_I2C_EN={0} ".format(args.test_i2c)
        if args.test_can == "1":
            fw_cmd += "TEST_CAN_EN={0} ".format(args.test_can)
        if args.test_timer == "1":
            fw_cmd += "TEST_TIMER_EN={0} ".format(args.test_timer)
        if args.test_rom == "1":
            fw_cmd += "TEST_ROM_EN={0} ".format(args.test_rom)
        if args.test_dcore == "1":
            fw_cmd += "TEST_DCORE_EN={0} ".format(args.test_dcore)
        if args.test_wdt == "1":
            fw_cmd += "TEST_WDT_EN={0} ".format(args.test_wdt)
        if args.test_bb == "1":
            fw_cmd += "TEST_BB_EN={0} ".format(args.test_bb)

    print(log)
    print("==========================================================")
    print(gen_header_fw_cmd)
    print(gen_header_bt_cmd)
    if args.test.lower() == "true":
        print("cd \\calterah\\baremetal\\")
        print(bt_cmd)
        print("cd ..\\..")
        print(fw_cmd)
        print(gen_header_fw_cmd)
        print(gen_header_bt_cmd)
    else:
        p1 = cmd_subprocess(fw_cwd, fw_cmd)
        p1.wait()
        if p1.returncode != 0:
            os._exit(1)
        #p2 = cmd_subprocess(bt_cwd, bt_cmd)
        #p2.wait()
        #if p2.returncode != 0:
        #    os._exit(1)
        p3 = cmd_subprocess(fw_cwd, gen_header_fw_cmd)
        p3.wait()
        if p3.returncode != 0:
            os._exit(1)

        #p4 = cmd_subprocess(fw_cwd, gen_header_bt_cmd)
        #p4.wait()
        #if p4.returncode != 0:
        #    os._exit(1)
        if args.stage == "3":
          print("\nBootloader processing ... ")
          if os.path.exists(bt_cwd):
            try:
              print("\tbuilding from ",bt_cwd)
              bt_cmd = "python make_bin.py -s {0} -m {1} -sp {2} -v {3} -aes {4} -tc {5} -tst {6} -c {7} -fw {8} -clean {9} -j {10} -bd {11} -app {12} -bm {13} -elf {14} -w2e {15} -xipload {16} -otaenc {17}".\
              format(args.stage, args.method, args.split, args.vendor, args.aes, args.toolchain, args.test, args.cascade, args.firmware, args.clean, args.thread, args.board, args.appl, args.bmethod,
              args.elfdump, args.warning2error, args.xipload, args.otaenc)
              p2 = cmd_subprocess(bt_cwd, bt_cmd)
              p2.wait()
              if p2.returncode != 0:
                os._exit(1)
            except (Exception,BaseException) as e:
              print("WARNING! Fail to build bootloader from ",bt_cwd)
              print("Copy bootloader from calterah/boot_prebuild/")
              bt_cmd = "cp -f calterah/boot_prebuild/*.bin ."
              cmd_subprocess(fw_cwd, bt_cmd)
          else:
            print("\tCopy bootloader from calterah/boot_prebuild/")
            bt_cmd = "cp -f calterah/boot_prebuild/*.bin ."
            cmd_subprocess(fw_cwd, bt_cmd)
        if args.combine == 'true':
            print("All in one process    : ",cmb_cmd)
            p1=cmd_subprocess(fw_cwd, cmb_cmd)
            p1.wait()
            if p1.returncode != 0:
                os._exit(1)

def check_args(args):
    logInfo = ''

    #toolchain check
    check = args.toolchain.lower()
    if check != 'gnu' and check !='mw':
      logInfo = 'Invalid param for -toolchain : %s\r\n' %args.toolchain
    else:
      args.toolchain = check

    #stage check
    check = args.stage.lower()
    if check != '2' and check !='3':
      logInfo += 'Invalid param for -stage : %s\r\n' %args.stage
    else:
      args.stage = check

    #method check
    check = args.method.lower()
    if check != 'ram' and check !='xip':
      logInfo += 'Invalid param for -method : %s\r\n' %args.method
    else:
      args.method = check

    #bbacc check
    check = args.bbacc.lower()
    if check != 'true' and check !='false':
      logInfo += 'Invalid param for -bbacc : %s\r\n' %args.bbacc
    else:
      args.bbacc = check

    #aes check
    check = args.aes.lower()
    if check != 'true' and check !='false':
      logInfo += 'Invalid param for -aes : %s\r\n' %args.aes
    else:
      args.aes = check

    #test check
    check = args.test.lower()
    if check != 'true' and check !='false':
        logInfo += 'Invalid param for -test : %s\r\n' %args.test
    else:
        args.test = check

    #otaenc check
    check = args.otaenc.lower()
    if check != 'true' and check !='false':
        logInfo += 'Invalid param for -otaenc : %s\r\n' %args.otaenc
    else:
        args.otaenc = check

    # Check HAL test cases, only if "-board=validation, -appl=validation"
    if args.board == 'validation':
        if args.test_uart == 'true':
            args.test_uart = '1'
        else:
            args.test_uart = '0'
        if args.test_gpio == 'true':
            args.test_gpio = '1'
        else:
            args.test_gpio = '0'
        if args.test_xip == 'true':
            args.test_xip = '1'
        else:
            args.test_xip = '0'
        if args.test_debug == 'true':
            args.test_debug = '1'
        else:
            args.test_debug = '0'
        if args.test_spi == 'true':
            args.test_spi = '1'
        else:
            args.test_spi = '0'
        if args.test_dma == 'true':
            args.test_dma = '1'
        else:
            args.test_dma = '0'
        if args.test_i2c == 'true':
            args.test_i2c = '1'
        else:
            args.test_i2c = '0'
        if args.test_can == 'true':
            args.test_can = '1'
        else:
            args.test_can = '0'
        if args.test_timer == 'true':
            args.test_timer = '1'
        else:
            args.test_timer = '0'
        if args.test_rom == 'true':
            args.test_rom = '1'
        else:
            args.test_rom = '0'
        if args.test_dcore == 'true':
            args.test_dcore = '1'
        else:
            args.test_dcore = '0'
        if args.test_wdt == 'true':
            args.test_wdt = '1'
        else:
            args.test_wdt = '0'
        if args.test_bb == 'true':
            args.test_bb = '1'
        else:
            args.test_bb = '0'
    else:
        args.test_uart = '0'
        args.test_gpio = '0'
        args.test_xip = '0'
        args.test_debug = '0'
        args.test_spi = '0'
        args.test_dma = '0'
        args.test_i2c = '0'
        args.test_can = '0'
        args.test_timer = '0'
        args.test_rom = '0'
        args.test_dcore = '0'
        args.test_wdt = '0'
        args.test_bb = '0'

    #cascade check
    check = args.cascade.lower()
    if check == 'true':
        args.combine = 'false'
    if len(logInfo)>0:
        print(logInfo)
        os._exit(-1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument('-stage', '-s', help = "Specify 2-stage or 3-stage boot [2/3]", default = "2")
    parser.add_argument('-method', '-m', help = "Specify method of executing [ram/xip]", default = "ram")
    parser.add_argument('-split', '-sp', help = "Split boot or not [true/false]", default = "false")
    parser.add_argument('-vendor', '-v', help = "Specify flash vendor for xip [giga/winbond/micron/microchip/mxic/s25fls/issi]", default = "s25fls")
    parser.add_argument('-bbacc', '-ba', help = "Specify baseband acceleration [true/false]", default = "false")
    parser.add_argument('-aes', help = "enable firmware aes [true/false]", default = "false")
    parser.add_argument('-toolchain', '-tc', help = "Specify toolchain [gnu/mw]", default = "gnu")
    parser.add_argument('-test','-tst', help = "Print compiling command [true/false]", default = "false")
    parser.add_argument('-cascade', '-c', help = "Specify whether firmware is used for cascade [true/false]", default = "false")
    parser.add_argument('-firmware','-fw', help = "Specify firmware version [debug/release]", default = "debug")
    parser.add_argument('-clean', help = "Clean project before compiling [true/false]", default = "true")
    parser.add_argument('-thread','-j', help = "Multi-thread compiling", default = "8")
    parser.add_argument('-hwcrc',help = "Whether to use HW_CRC for boot", default = "1")
    parser.add_argument('-combine','-cmb', help = "to combine header + bootloader[optional] + firmware to one image [true/false].", default = "true")
    parser.add_argument('-board',  '-bd',  help = "board config", default="ref_design" )
    parser.add_argument('-appl',   '-app', help = "application name", default="sensor" )
    parser.add_argument('-bmethod', '-bm', help = "Specify method of bootloader executing [ram/xip]", default = "ram")
    parser.add_argument('-timefolder','-tf', help = "output folder with time stamp [true/false]", default = "false")
    parser.add_argument('-elfdump','-elf', help = "collect ELF and dump files to output folder [true/false]", default = "true")
    parser.add_argument('-warning2error','-w2e', help = "warning to error [true/false]", default = "false")
    parser.add_argument('-xipload',help = "bootloader use XIP to load FW for 3-stage boot[true/false]", default = "true")

    # Below was used by -board=validation, -appl=validation
    parser.add_argument('-test_uart','-uart', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_gpio','-gpio', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_xip','-xip', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_debug','-debug', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_spi','-spi', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_dma','-dma', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_i2c','-i2c', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_can','-can', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_timer','-timer', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_rom','-rom', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_dcore','-dcore', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_wdt','-wdt', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-test_bb','-bb', help = "Specify test module is on or off [true/false].", default = "false")
    parser.add_argument('-macro', nargs='*')
    parser.add_argument('-otaenc', help = "Crypto OTA functions for security, true for enable; false for disable [true/false]", default = "false")
    args = parser.parse_args()

    #check args first
    check_args(args)

    main(args)