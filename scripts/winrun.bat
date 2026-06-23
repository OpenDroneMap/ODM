@echo off

setlocal

cd /d "%~dp0"
call win32env.bat
python -X utf8 "%ODMBASE%\run.py" %*

endlocal

if defined ODM_NONINTERACTIVE (
    exit
)