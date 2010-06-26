@echo off
rem ---------------------------------------------
rem Update SVN revision include and batch files
rem ---------------------------------------------

if exist UAVXRxRevision.h del UAVXRxRevision.h
SubWCRev . UAVXRxRevisionSVN.h UAVXRxRevision.h

if exist makeallhelper.bat del makeallhelper.bat
SubWCRev . makeallhelperSVN.bat makeallhelper.bat
