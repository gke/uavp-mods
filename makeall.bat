@echo off
rem To see why we do the setlocal, see:
rem http://www.robvanderwoude.com/variableexpansion.html
rem http://www.robvanderwoude.com/ntset.html
SETLOCAL ENABLEDELAYEDEXPANSION

rem Batch compiles various possibilities of the UFO test software
rem
rem Greg Egan 2008
rem
rem Uses: makeallhelper.bat and makeclean.bat
rem 
rem Version of the board. May be 3_0 or 3_1.
rem Type of gyros in use. May be OPT_ADXRS or IDG300.
rem Type of ESC in use. May be ESC_PWM,  ESC_HOLGER.
rem Type of Rx. May RX_PPM for serial composite PPM frame, or 
rem RX_DEFAULT for default PPM Graupner/JR etc Rx.

set VERSION=3_14m3

rem Add/Delete required combinations to these sets
set BOARDS= 3_1
set GYROS=OPT_IDG OPT_ADXRS
set ESCS=ESC_PPM ESC_HOLGER
set DBGS=NO_DEBUG 
set RXS=RX_DEFAULT RX_PPM

rem Personal choice


rem Delete working files
call makeclean.bat
echo Starting makeall uavp > gen.lst
echo Starting makeall uavp > log.lst

rem Parameters for makeallhelper.bat are VERSION BOARD GYRO ESC DBG RX

for %%b in (%BOARDS%) do for %%g in (%GYROS%) do for %%e in (%ESCS%) do for %%d in (%DBGS%) do for %%r in (%RXS%) do call makeallhelper.bat %VERSION% %%b %%g %%e %%d %%r 




