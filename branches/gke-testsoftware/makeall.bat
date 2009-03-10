@echo off
rem To see why we do the setlocal, see:
rem http://www.robvanderwoude.com/variableexpansion.html
rem http://www.robvanderwoude.com/ntset.html
SETLOCAL ENABLEDELAYEDEXPANSION

rem Batch compiles various possibilities of the UFO software for the Microchip 18F2xxx series
rem
rem Greg Egan 2008-2009
rem
rem Uses: makeallhelper.bat and makeclean.bat
rem 
rem Type of gyros in use. May be OPT_ADXRS or IDG300.
rem Type of ESC in use. May be ESC_PPM,  ESC_HOLGER.
rem Type of Rx. May be RX_PPM for serial PPM frame, or RX_DEFAULT.
rem Type of debugging to use. NO_DEBUG with test software.
rem Configuration is the same for quads and tricopters.

set VERSION=3_14m3_18f

rem Add/Delete required combinations to these sets
set GYRO=OPT_IDG OPT_ADXRS
set ESC=ESC_PPM ESC_HOLGER
set DBG=NO_DEBUG
set RX=RX_DEFAULT RX_PPM
set CFG=QUAD

rem Personal choice
rem set GYRO=OPT_IDG OPT_ADXRS
rem set ESC=ESC_PPM
rem set DBG=NO_DEBUG
rem set RX=RX_DEFAULT RX_PPM
rem set CFG=

rem Delete working files
call makeclean.bat
echo Starting makeall uavp > gen.lst
echo Starting makeall uavp > log.lst

rem Parameters for makeallhelper.bat are VERSION BARO GYRO ESC DBG RX

for %%g in (%GYRO%) do for %%e in (%ESC%) do for %%r in (%RX%) do call makeallhelper.bat %VERSION% %%g %%e %%r




