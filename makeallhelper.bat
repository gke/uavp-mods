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
set 	RX=%4

set CSRC=an-test i2c-scan irq lisl adc output pu-test rxtest serial text utils utils2
set ASRC=bootl18f2520

set CC="C:\MCC18\bin\mcc18"
set CCMD=  -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-  -DBATCHMODE -DNOLEDGAME 

set ACMD=/o+ /e+ /l+ /x- /p18f2520 /c+ /q
set AEXE="%ProgramFiles%\microchip\MPASM Suite\MPASMwin.exe"

set LCMD=/p18F2520 /l"C:\MCC18\lib" /k"C:\MCC18\lkr"
set LEXE="C:\MCC18\bin\mplink.exe"

rem Set all the name tokens for the HEX files
set G=
set E=
set D=
set T=
set R=
set B=
set C=
if "%GYRO%"  == "OPT_ADXRS"      set G=ADXRS-
if "%GYRO%"  == "OPT_IDG"           set G=IDG-
if "%ESC%"   == "ESC_PPM"           set E=PPM
if "%ESC%"   == "ESC_HOLGER"        set E=HOL
if "%RX%"    == "RX_PPM"            set R=RXCOM-

rem Build the list of expected object files
set F=
for %%i in ( %CSRC% ) do set F=!F! %%i.o
for %%i in ( %ASRC% ) do set F=!F! %%i.o

rem The warnings etc. previously directed to NUL have been reinstated to log.lst. These 
rem include a number associated with argument passing other than by function parameters to
rem the mathematics module.
rem The local variable offset -ro1 is to overcome aliasing of variables caused by cc5x!
rem As a consequence there are several warnings on bank allocation in the compile.

for %%i in ( %CSRC% ) do %CC% -p=18F2520 /i"C:\MCC18\h" %%i.c -fo=%%i.o %CCMD% -D%GYRO% -D%ESC% -D%RX%  >> log.lst

for %%i in ( %ASRC% ) do %AEXE%  %%i.asm %ACMD% >> log.lst

%LEXE% %LCMD% %F% /u_CRUNTIME /z__MPLAB_BUILD=1 /W /o UAVPTest-V%VERSION%-%T%%G%%R%%E%.hex >> log.lst 


if %ERRORLEVEL% == 1 goto FAILED

echo compiled - UAVPTest-V%VERSION%-%D%%T%%G%%R%%E%.hex
echo compiled - UAVPTest-V%VERSION%-%D%%T%%G%%R%%E%.hex >> gen.lst
call makeclean.bat
goto FINISH

:FAILED
echo failed - UAVPTest-V%VERSION%-%T%%G%%R%%E%.hex
echo failed - UAVPTest-V%VERSION%-%T%%G%%R%%E%.hex >> gen.lst
rem don't delete working files

:FINISH








