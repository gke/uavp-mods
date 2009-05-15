@echo off
rem -----------------------------------------------------
rem Update SVN revision number in include and batch files
rem -----------------------------------------------------

if exist c-ufo-revision.h del c-ufo-revision.h
SubWCRev . c-ufo-revision-SVN.h c-ufo-revision.h

if exist makeallhelper.bat del makeallhelper.bat
SubWCRev . makeallhelperSVN.bat makeallhelper.bat




