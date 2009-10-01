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
rem Clock rate CLOCK_16MHZ (Only 16MHZ available for UAVP version)
rem Type of PIC processor 18F2620 only
rem DEBUG_SENSORS to generate trace files of all main program and sensor values which can be plotted
rem using UAVPSet (blank option in menu below testsoftware).
rem Configuration TRICOPTER for three motor and QUAD for 4.
rem Motors are disabled for DEBUG_SENSORS for safety reasons.
rem Prevous switches THC and CAM have been abandoned in favour of Tx based
rem throttle shaping and X-mode to orient the camera forward set under UAVPSet.

set VERSION=1_0m3_

rem Add/Delete required combinations to these sets
set CLOCK=CLOCK_16MHZ
set PROC=18F2620
set DBG=NO_DEBUG DEBUG_SENSORS 
set RX=RX7CH RX6CH
set CFG=QUAD TRICOPTER

rem Personal choice
rem set CLOCK=CLOCK_16MHZ
rem set PROC=18F2520
rem set DBG=NO_DEBUG DEBUG_SENSORS
rem set RX=RX7CH RX6CH
rem set CFG=QUAD

rem Delete working files
call makeclean.bat

rem Requires Tortoise SVN 
call makerev.bat

del *.HEX

echo Starting makeall uavp > gen.lst
echo Starting makeall uavp > log.lst

rem Parameters for makeallhelper.bat are VERSION BARO GYRO ESC DBG RX

for %%x in (%CLOCK%) do for %%p in (%PROC%) do for %%d in (%DBG%) do for %%r in (%RX%) do for %%c in (%CFG%) do call makeallhelper.bat %VERSION% %%x %%p %%d  %%r %%c

