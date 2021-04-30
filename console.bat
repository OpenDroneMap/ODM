@echo off

setlocal
call win32env.bat

start "ODM Console" cmd "/k venv\Scripts\activate"

endlocal
