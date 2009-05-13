@echo off
rem ---------------------------------------------
rem Update SVN revision include file c-ufo-revision.h
rem ---------------------------------------------

if exist c-ufo-revision.h del c-ufo-revision.h
SubWCRev . c-ufo-revision-SVN.h c-ufo-revision.h



