@echo off
rem To see why we do the setlocal, see:
rem http://www.robvanderwoude.com/variableexpansion.html
rem http://www.robvanderwoude.com/ntset.html
SETLOCAL ENABLEDELAYEDEXPANSION

rem Batch compiles various possibilities of the UFO software
rem
rem Greg Egan 2008
rem
rem Uses: makeallhelper.bat and makeclean.bat
rem 
rem Version of the board. May be 3_0 or 3_1.
rem Type of gyros in use. May be OPT_ADXRS300, OPT_ADXRS150, or IDG300.
rem Type of ESC in use. May be ESC_PPM,  ESC_YGEI2C, ESC_HOLGER.
rem Type of Rx. May be RX_AR7000 for Spektrum Rx, RX_PPM for serial PPM frame,
rem or RX_DEFAULT for default PPM Graupner/JR etc Rx
rem Throttle curve USE_THROTTLECURVE for possibly better control around hover.
rem Type of debugging to use. May be DEBUG_MOTORS (3.1 only) or DEBUG_SENSORS.
rem Motors are disabled for DEBUG_SENSORS as there is not enough program space
rem in the 876 PIC.

set VERSION=3_15m3

rem Add/Delete required combinations to these sets
set BOARDS= 3_0 3_1
set GYROS=OPT_IDG OPT_ADXRS150 OPT_ADXRS300
set ESCS=ESC_PPM ESC_HOLGER ESC_YGEI2C
set DBGS=NO_DEBUG DEBUG_SENSORS DEBUG_MOTORS
set THCS=NO_THROTTLECURVE USE_THROTTLECURVE 
set RXS=RX_DEFAULT RX_PPM RX_AR7000

rem Personal choice
set BOARDS= 3_1
set GYROS=OPT_IDG
set ESCS=ESC_PPM
set DBGS=NO_DEBUG
set THCS=NO_THROTTLECURVE 
set RXS=RX_DEFAULT

rem Parameters for makeallhelper.bat are VERSION BOARD GYRO ESC DBG THC RX

set BOARD=3_1
for %%b in (%BOARDS%) do for %%g in (%GYROS%) do for %%e in (%ESCS%) do for %%d in (%DBGS%) do for %%t in (%THCS%) do for %%r in (%RXS%) do call makeallhelper.bat %VERSION% %%b %%g %%e %%d %%t %%r 

rem set BOARD=3_0
rem for %%g in (%GYROS%) do for %%e in (%ESCS%) do for %%d in (%DBGS%) do for %%t in (%THCS%) do for %%r in (%RXS%) do call makeallhelper.bat %VERSION% %%BOARDS %%g %%e %%d %%t %%r 





