@echo off
rem Bypass "Terminate Batch Job" prompt.

setlocal
cd %~dp0
winrun.bat %* <NUL	
endlocal