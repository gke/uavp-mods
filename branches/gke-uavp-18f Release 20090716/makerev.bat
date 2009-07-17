@echo off
rem ---------------------------------------------
rem Update SVN revision include and batch files
rem ---------------------------------------------

if exist UAVPRevision.h del UAVPRevision.h
SubWCRev . UAVPRevisionSVN.h UAVPRevision.h

if exist makeallhelper.bat del makeallhelper.bat
SubWCRev . makeallhelperSVN.bat makeallhelper.bat

if exist makealltesthelper.bat del makealltesthelper.bat
SubWCRev . makealltesthelperSVN.bat makealltesthelper.bat