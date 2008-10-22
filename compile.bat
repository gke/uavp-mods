@echo off
rem Batch compiliert diverse Möglichkeiten der Ufo-Software
rem =======================================================
rem Schalter:
rem NOLEDGAME immer
rem USE_ACCSENS immer
rem OPT_ADXRS150 + OPT_ADXRS300 + OPT_IDG
rem ESC_PPM + ESC_HOLGER
rem BOARD_3_0 + BOARD_3_1
rem DEBUG_MOTORS (nur bei BOARD_3_1)

set CSRC=accel c-ufo irq lisl mathlib matrix pid pid2 prog sensor serial utils utils2
set ASRC=bootloader

set CEXE="%ProgramFiles%\microchip\cc5x\cc5x.exe"
set CCMD=-CC -p16F876 -I"%ProgramFiles%\microchip\cc5x" -a -L -Q -V -FM +reloc.inc -DMATHBANK_VARS=0 -DMATHBANK_PROG=2 -DBATCHMODE -DNOLEDGAME -DUSE_ACCSENS -X
set ACMD=/o+ /e+ /l+ /x- /p16F876 /c+ /q
set AEXE="%ProgramFiles%\microchip\MPASM Suite\MPASMwin.exe"
set LCMD=16f876i.lkr /aINHX8M
set LEXE="%ProgramFiles%\microchip\MPASM Suite\mplink.exe"

set ZIP="%ProgramFiles%\IZarc\IZarcC.exe" -a

set VS=3.15
set VG=3.09

rem Als erstes Testen ob cmd mit /v aufgerufen wurde
set F=x
for %%i in (a b) do set F=!F! %%i
if  "%F%" == "x b"  goto CMDERR

rem Fuer die Liste der erzeugten Pakete
set OF=

echo Loeschen der C-Ausgabe-Dateien...
for %%i in ( %CSRC% ) do if exist %%i.asm del %%i.asm
for %%i in ( %CSRC% ) do if exist %%i.err del %%i.err
for %%i in ( %CSRC% ) do if exist %%i.fcs del %%i.fcs
for %%i in ( %CSRC% ) do if exist %%i.lst del %%i.lst
for %%i in ( %CSRC% ) do if exist %%i.o   del %%i.o  
for %%i in ( %CSRC% ) do if exist %%i.occ del %%i.occ
for %%i in ( %CSRC% ) do if exist %%i.var del %%i.var

echo Loeschen der ASM-Ausgabe-Dateien...
for %%i in ( %ASRC% ) do if exist %%i.err del %%i.err
for %%i in ( %ASRC% ) do if exist %%i.lst del %%i.lst
for %%i in ( %ASRC% ) do if exist %%i.o   del %%i.o

echo Loeschen von HEX-Files...
if exist profi-ufo.cod del profi-ufo.cod
if exist profi-ufo.hex del profi-ufo.hex
if exist profi-ufo.lst del profi-ufo.lst
if exist profi-ufo.map del profi-ufo.map

rem Das folgende wird 2x durchlaufen!

SET CAM=CAM_0_DEG
SET RX=

:STEP01
set NEXT=ENDE
rem =============================================
rem C-Compiling, assembling and linking a version
rem =============================================

set BOARD=3_1
set GYRO=OPT_ADXRS300
set ESC=ESC_PPM
rem set DBG=NODEBUG_MOTORS
set DBG=DEBUG_SENSORS
set THC=NO_THROTTLECURVE
goto DOIT

:DOIT
echo.
echo.
echo.
echo.
echo NEXT= %NEXT%
echo.
for %%i in ( %CSRC% ) do call helper.bat CC5X %%i.c   -DBOARD_%BOARD% -D%GYRO% -D%ESC% -D%DBG% -D%THC% -D%CAM% -D%RX%
for %%i in ( %ASRC% ) do call helper.bat ASM  %%i.asm /dBOARD_%BOARD%

rem =============================================
rem = set all the name tokens for the HEX files
rem =============================================
set G=
set E=
set V=
set D=
set T=
set C=CAM0-
set R=
if "%GYRO%"  == "OPT_ADXRS300"      set G=ADX300-
if "%GYRO%"  == "OPT_ADXRS150"      set G=ADX150-
if "%GYRO%"  == "OPT_IDG"           set G=IDG-
if "%ESC%"   == "ESC_PPM"           set E=PPM
if "%ESC%"   == "ESC_HOLGER"        set E=HOL
if "%ESC%"   == "ESC_X3D"           set E=X3D
if "%ESC%"   == "ESC_YGEI2C"        set E=YGE
if "%BOARD%" == "3_1"	            set V=%VS%
if "%BOARD%" == "3_0"	            set V=%VG%
if "%DBG%"   == "DEBUG_MOTORS"      set D=DBG-
if "%DBG%"   == "DEBUG_SENSORS"     set D=SEN-
if "%THC%"   == "USE_THROTTLECURVE" set T=THC-
if "%CAM%"   == "CAM_45_DEG"        set C=CAM45-
if "%RX%"    == "RX_PPM"            set R=RXCOM-

echo Linke Profi-Ufo-V%V%-%D%%T%%C%%G%%R%%E%
%LEXE% %LCMD% *.o /o Profi-Ufo-V%V%-%D%%T%%C%%G%%R%%E%.hex
if %ERRORLEVEL% == 1 goto ENDE
del Profi-Ufo-V%V%-%D%%T%%C%%G%%R%%E%.cod
set OF=!OF! Profi-Ufo-V%V%-%D%%T%%C%%G%%R%%E%

goto %NEXT%

:SYNTAX
echo Fehler!
echo Aufruf mit MAKEUFO BBB GGG
echo wobei BBB die Versionsnummer der schwarzen und GGG der gruenen Platine ist!
goto ENDE

:CMDERR
echo Fehler!
echo CMD wurde nicht mit /V aufgerufen!

:ENDE
