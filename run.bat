@echo off
rem Bypass "Terminate Batch Job" prompt.

setlocal

cd /d %~dp0
winrun.bat %* <NUL

endlocal
