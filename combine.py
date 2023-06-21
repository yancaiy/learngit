#!/usr/bin/env python3

import os, sys,time,shutil,argparse
from git.repo import Repo

chip = "alpsMP"

if __name__ == '__main__' :

    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument('-board',  '-bd',  help = "board config", default="ref_design" )
    parser.add_argument('-appl',   '-app', help = "application name", default="ref_design" )
    parser.add_argument('-timefolder','-tf', help = "output folder with time stamp[true/false]", default="true" )
    parser.add_argument('-elfdump','-elf', help = "collect ELF and dump files to output folder[true/false]", default="false" )
    parser.add_argument('-toolchain', '-tc', help = "Specify toolchain [gnu/mw]", default = "gnu")
    parser.add_argument('-vendor', '-v', help = "Specify vendor [giga/s25fls]", default = "giga")
    parser.add_argument('-stage', '-s', help = "Specify 2-stage or 3-stage boot [2/3]", default = "2")
    parser.add_argument('-aes', help = "enable firmware aes [true/false]", default = "false")
    parser.add_argument('-method', '-m', help = "Specify method of executing [ram/xip]", default = "ram")
    parser.add_argument('-bmethod', '-bm', help = "Specify method of bootloader executing [ram/xip]", default = "ram")

    args = parser.parse_args()

    names = ["header.bin","boot.bin","firmware.bin","firmware_combine.bin"]

    if os.path.isfile(names[0]) != True :
        print("""\nPlease check header file [ header.bin ] first!""")
        os._exit(-1)

    # Check git ID
    git_rev ='git-none'
    try:
        repo = Repo(os.getcwd())
        git_rev="g"+repo.git.rev_parse('--verify','--short','HEAD')
        checklist=repo.git.diff_index('--name-only', 'HEAD')
        checklist=checklist.strip()
        if len(checklist) > 0 :
            git_rev=git_rev+"-dirty"
    except (Exception,BaseException) as e:
        ##print('\r\n\t\t\t WARNING! Git init failed in folder [',e,']\r\n')
        git_rev ='git-none'

    # Generate folder name
    if (args.timefolder == "true"):
        new_dir+=time.strftime("-%Y%m%d%H%M%S", time.localtime())

    new_dir=args.appl+'_'+git_rev+'_'+args.vendor

    if args.stage == "2":
        new_dir+="-2-stage-"
    else:
        new_dir+="-3-stage-{0}-".format(args.bmethod)
    if args.method == "ram":
        new_dir+="ram"
    elif args.method == "xip":
        new_dir+="xip"
    else:
        new_dir+="unknow"

    new_dir +='_'+args.toolchain

    # print(new_dir),print("\n\n")
    time_dir = time.strftime("autobuild/%Y-%m-%d/", time.localtime())
    if os.path.isdir(time_dir) == False  :
        os.makedirs(time_dir,0o777,True)

    new_dir = time_dir + new_dir;

    if os.path.isdir(new_dir) == True  :
        shutil.rmtree(new_dir,True)

    # Copy binary files
    os.makedirs(new_dir,0o777,True)
    idx = 0
    while (idx < len(names)) :
        if os.path.isfile(names[idx]) == True :
            # print("Copied: ", names[idx])
            shutil.copy(names[idx],new_dir)
        idx =idx + 1

    # Copy resource files: elf, map, dump
    res_dir = new_dir + "/resource"
    if os.path.isdir(res_dir) == False  :
        os.makedirs(res_dir,0o777,True)
    if ('true' == args.elfdump.lower()):
        if ("3" == args.stage):
            bootloader="calterah/bootloader/baremetal/obj_" + chip + "_{0}_v1_{1}/{2}_arcem6/{3}_{4}_arcem6".format('ref_design','boot',args.toolchain,'boot',args.toolchain)
            shutil.copy(bootloader+".elf",res_dir)
            shutil.copy(bootloader+".dump",res_dir)
            shutil.copy(bootloader+".map",res_dir)
        firmware="obj_" + chip + "_{0}_v1_{1}/{2}_arcem6/{3}_{4}_arcem6".format(args.board,args.appl,args.toolchain,args.appl,args.toolchain)
        shutil.copy(firmware+".elf",res_dir)
        shutil.copy(firmware+".dump",res_dir)
        shutil.copy(firmware+".map",res_dir)
    # shutil.move(new_dir+'/'+names[0],res_dir)
    # print("\t\t\t", names[0], " is generated to ", new_dir)
    os._exit(0)
