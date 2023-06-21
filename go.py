#!/usr/bin/env python3
import os,sys,subprocess,time,argparse

#building check with dedicated projects to verify the code update
#v0.1 2021.05.18
#The total is 24 BSPs but with normal/AES enabled we can get 48 buildings
#1   : -- pass  -- [ python make_bin.py -tf false -both true -s 3  -bm ram  -m ram  -board ref_design -appl ref_design  -toolchain gnu  ]
#2   : -- fail  -- [ python make_bin.py -tf false -both true -s 3  -bm ram  -m ram  -board ref_design -appl ref_design  -toolchain mw  ]
#....

def main(args):

    #current working folder
    fw_cwd = os.getcwd()

    #stage type:2/3
    stageT = [" -s 3 ", " -s 2 "]
    #bootloader mode: ram/xip
    bmodeT = [" -bm ram "," -bm xip "]
    #firmware mode : ram/xip
    fmodeT = [" -m ram ", " -m xip "]
    #board/application name: ref_design/validation
    nameT  = [" -board ref_design -appl sensor "," -board validation -appl validation "]
    #toolchain : gnu/mw
    toolT  = [" -toolchain gnu "," -toolchain mw "]

    #output folder check & create
    time_dir = time.strftime("autobuild/%Y-%m-%d/", time.localtime())
    if os.path.isdir(time_dir) == False  :
        os.makedirs(time_dir,0o777,True)

    #build log file name
    logFN = time_dir + "build_check.log"
    fileLog = open(logFN,"a+t")
    #time stamp for building start
    loginfo = time.strftime("\nBegin : %Y-%m-%d-%H:%M:%S\n\n", time.localtime())
    fileLog.write(loginfo)
    fileLog.flush()
    print(loginfo)
    index = 1
    totalLog = loginfo
    for tkname in nameT :
        for tkstage in stageT :
            for tkfmode in fmodeT :
                for tktool in toolT :
                    basecmd = "python make_bin.py -tf false -test {0} -v {1} -warning2error {2}".format(args.test, args.vendor, args.warning2error) + tkstage
                    if (tkstage == " -s 2 ") :
                        cmd = basecmd + tkfmode + tkname + tktool
                        print(cmd)
                        print(fw_cwd)
                        process = subprocess.Popen(cmd, cwd=fw_cwd, shell=True)
                        process.wait()
                        result="fail"
                        if process.returncode == 0 :
                             result="pass"
                        loginfo ="{:<3} : -- {:^5} -- [ {:<} ]\r\n".format(index,result,cmd)
                        totalLog += loginfo
                        index +=1
                        print(totalLog)
                        fileLog.write(loginfo)
                        fileLog.flush()
                    else :
                        for tkbmode in bmodeT :
                            cmd = basecmd + tkbmode + tkfmode + tkname + tktool
                            print(cmd)
                            print(fw_cwd)
                            process = subprocess.Popen(cmd, cwd=fw_cwd, shell=True)
                            process.wait()
                            result="fail"
                            if process.returncode == 0 :
                                 result="pass"
                            loginfo ="{:<3} : -- {:^5} -- [ {:<} ]\r\n".format(index,result,cmd)
                            totalLog +=loginfo
                            index +=1
                            print(totalLog)
                            fileLog.write(loginfo)
                            fileLog.flush()

    #time stamp for building done
    loginfo = time.strftime("\nEnd : %Y-%m-%d-%H:%M:%S\n", time.localtime())
    fileLog.write(loginfo)
    print(loginfo)

    index-=1
    summary="\n----------------------Total {0} + {1} BSP build done----------------------\n\n".format(index,index)
    print(summary)
    fileLog.write(summary)
    fileLog.close()



def check_args(args):
    logInfo = ''

    #test check
    check = args.test.lower()
    if check != 'true' and check !='false':
      logInfo += 'Invalid param for -test : %s\r\n' %args.test
    else:
      args.test = check

    #warning to error check
    check = args.warning2error.lower()
    if check != 'true' and check !='false':
      logInfo += 'Invalid param for -w2e : %s\r\n' %args.warning2error
    else:
      args.warning2error = check

    if len(logInfo)>0:
      print(logInfo)
      os._exit(-1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument('-vendor', '-v',  help = "Specify flash vendor for xip [giga/winbond/micron/microchip/mxic/s25fls]",default="s25fls")
    parser.add_argument('-test', '-tst',  help = "Print command", default = "false")
    parser.add_argument('-warning2error','-w2e', help = "warning to error [true/false]", default = "false")

    args = parser.parse_args()

    #check args first
    check_args(args)

    #main process
    main(args)