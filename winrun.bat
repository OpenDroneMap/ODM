@echo off

setlocal

call win32env.bat
python "%ODMBASE%\run.py" %*

endlocal

