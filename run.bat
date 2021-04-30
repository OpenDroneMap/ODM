@echo off

setlocal

call win32env.bat
call %ODMBASE%\venv\Scripts\activate

python "%ODMBASE%\run.py" %*

endlocal

