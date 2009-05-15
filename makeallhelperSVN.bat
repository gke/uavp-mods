@echo off
rem To see why we do the setlocal, see:
rem http://www.robvanderwoude.com/variableexpansion.html
rem http://www.robvanderwoude.com/ntset.html
SETLOCAL ENABLEDELAYEDEXPANSION

rem Helper script for makeall.bat
rem =======================================================
rem parameters passed are:
set 	VERSION=%1
set 	GYRO=%2
set 	ESC=%3
set 	DBG=%4
set 	RX=%5
set 	CFG=%6

for /f "tokens=2-4 delims=/ " %%a in ('date /T') do set year=%%c
for /f "tokens=2-4 delims=/ " %%a in ('date /T') do set month=%%a
for /f "tokens=2-4 delims=/ " %%a in ('date /T') do set day=%%b
set TODAY=%year%%month%%day%

for /f "tokens=1 delims=: " %%h in ('time /T') do set hour=%%h
for /f "tokens=2 delims=: " %%m in ('time /T') do set minutes=%%m
for /f "tokens=3 delims=: " %%a in ('time /T') do set ampm=%%a
set NOW=%hour%%minutes%%ampm%

set CSRC=accel c-ufo irq lisl mathlib matrix pid pid2 prog sensor serial utils utils2
set ASRC=bootloader

set CEXE="%ProgramFiles%\microchip\cc5x\cc5x.exe"
set CCMD=-CC -p16F876 -I"%ProgramFiles%\microchip\cc5x" -a -L -Q -V -FM +reloc.inc -DMATHBANK_VARS=0 -DMATHBANK_PROG=2 -DBATCHMODE -DNOLEDGAME -DUSE_ACCSENS -X
set ACMD=/o+ /e+ /l+ /x- /p16F876 /c+ /q
set AEXE="%ProgramFiles%\microchip\MPASM Suite\MPASMwin.exe"
set LCMD=16f876i.lkr /aINHX8M
set LEXE="%ProgramFiles%\microchip\MPASM Suite\mplink.exe"

rem Set all the name tokens for the HEX files
set G=
set E=
set D=
set T=
set R=
set B=
set C=
if "%GYRO%"  == "OPT_ADXRS300"      set G=ADX300-
if "%GYRO%"  == "OPT_ADXRS150"      set G=ADX150-
if "%GYRO%"  == "OPT_IDG"           set G=IDG-
if "%ESC%"   == "ESC_PPM"           set E=PPM
if "%ESC%"   == "ESC_HOLGER"        set E=HOL
if "%ESC%"   == "ESC_X3D"           set E=X3D
if "%ESC%"   == "ESC_YGEI2C"        set E=YGE
if "%DBG%"   == "DEBUG_MOTORS"      set D=Debug_MOTORS-
if "%DBG%"   == "DEBUG_SENSORS"     set D=Debug_SENSORS-
if "%RX%"    == "RX_PPM"            set R=RXCOM-
if "%RX%"    == "RX_DSM2"           set R=DSM2-
if "%CFG%"    == "TRICOPTER"           set C=TRI-

rem Build the list of expected object files
set F=
for %%i in ( %CSRC% ) do set F=!F! %%i.o
for %%i in ( %ASRC% ) do set F=!F! %%i.o

rem The warnings etc. previously directed to NUL have been reinstated to log.lst. These 
rem include a number associated with argument passing other than by function parameters to
rem the mathematics module.
rem The local variable offset -ro1 is to overcome aliasing of variables caused by cc5x!
rem As a consequence there are several warnings on bank allocation in the compile.

for %%i in ( %CSRC% ) do %CEXE% %%i.c  %CCMD% -D%GYRO% -D%ESC% -D%DBG% -D%RX% -D%CFG%>> log.lst
rem recompiling sensor.c with -r01 to avoid the use of a separate batch file with conditionals.
%CEXE% sensor.c  %CCMD% -DBOARD_%BOARD% -D%GYRO% -D%ESC% -D%DBG% -D%RX% -D%CFG% -ro1 >> log.lst 

for %%i in ( %ASRC% ) do %AEXE%  %%i.asm %ACMD% >> log.lst

%LEXE% %LCMD% %F% /o UAVP-V3.15.$WCREV$gke-16F876-16_%C%%D%%T%%G%%R%%E%.hex >> log.lst 


if %ERRORLEVEL% == 1 goto FAILED

echo compiled - UAVP-V3.15.$WCREV$gke-16F876-16_%C%%D%%T%%G%%R%%E%.hex
echo compiled - UAVP-V3.15.$WCREV$gke-16F876-16_%C%%D%%T%%G%%R%%E%.hex >> gen.lst
call makeclean.bat
goto FINISH

:FAILED
echo failed - UAVP-V3.15.$WCREV$gke-16F876-16_%C%%D%%T%%G%%R%%E%.hex
echo failed - UAVP-V3.15.$WCREV$gke-16F876-16_%C%%D%%T%%G%%R%%E%.hex >> gen.lst
rem don't delete working files

:FINISH








