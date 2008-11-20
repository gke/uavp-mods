@echo off
rem To see why we do the setlocal, see:
rem http://www.robvanderwoude.com/variableexpansion.html
rem http://www.robvanderwoude.com/ntset.html
SETLOCAL ENABLEDELAYEDEXPANSION

rem Batch compiles various possibilities of the UFO software
rem Greg Egan 2008
rem Uses: makeallhelper.bat and makeclean.bat
rem =======================================================
rem Version of the board. May be 3_0 or 3_1.
rem Type of gyros in use. May be OPT_ADXRS300, OPT_ADXRS150, or IDG300.
rem Type of ESC in use. May be ESC_PPM,  ESC_YGEI2C, ESC_HOLGER, etc...
rem Type of Rx. RX_AR7000 for Spektrum Rx, RX_PPM for serial PPM frame,
rem or RX_DEFAULT for default PPM Graupner/JR etc Rx
rem Throttle curve USE_THROTTLECURVE for better control around hover.
rem Type of debugging to use. May be DEBUG_MOTORS (3.1 only) or DEBUG_SENSORS.
rem Motors are disabled for DEBUG_SENSORS as there is not enough program space.

set VERSION=m3 

rem Add required combinations to these sets
set BOARDS= 3_0 3_1
set GYROS=OPT_IDG OPT_ADXRS150 OPT_ADXRS300
set ESCS=ESC_PPM ESC_HOLGER ESC_YGEI2C
set DBGS=NO_DBG DBG_SENSOR DBG_MOTOR
set THCS=NO_THROTTLECURVE USE_THROTTLECURVE 
set CAMS=CAM_0_DEG CAM_45_DEG
set RXS=RX_DEFAULT RX_PPM RX_AR7000

rem Parameters for makeallhelper.bat are VERSION BOARD GYRO ESC DBG THC CAM RX

set DBG=NO_DBG
set BOARD=3_1
for %%g in (%GYROS%) do for %%e in (%ESCS%) do for %%t in (%THCS%) do for %%c in (%CAMS%) do for %%r in (%RXS%) do call makeallhelper.bat %VERSION% %BOARD% %%g %%e %DBG% %%t %%c %%r 

set DBG=DBG_SENSORS
set BOARD=3_0
rem for %%g in (%GYROS%) do for %%e in (%ESCS%) do for %%t in (%THCS%) do for %%c in (%CAMS%) do for %%r in (%RXS%) do call makeallhelper.bat %VERSION% %BOARD% %%g %%e %DBG% %%t %%c %%r 





