
@echo off
@rem "Build the boot and firmware"

make clean
make -C calterah\baremetal clean

python.exe make_bin.py -s 2 -m ram -v s25fls

md build
move *.bin build\
move *.dat build\
xcopy /s /e /i /y autobuild build\release
rd /s /q autobuild

pause
