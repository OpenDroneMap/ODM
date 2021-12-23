@echo off

rem This file is UTF-8 encoded, so we need to update the current code page while executing it
for /f "tokens=2 delims=:." %%a in ('"%SystemRoot%\System32\chcp.com"') do (
    set _OLD_CODEPAGE=%%a
)
if defined _OLD_CODEPAGE (
    "%SystemRoot%\System32\chcp.com" 65001 > nul
)

set ODMBASE=%~dp0
set GDALBASE=%ODMBASE%venv\Lib\site-packages\osgeo
set OSFMBASE=%ODMBASE%SuperBuild\install\bin\opensfm\bin
set SBBIN=%ODMBASE%SuperBuild\install\bin

set PATH="%GDALBASE%";"%SBBIN%";"%OSFMBASE%"
set PROJ_LIB=%GDALBASE%\data\proj

set VIRTUAL_ENV=%ODMBASE%venv
set PYTHONPATH=%VIRTUAL_ENV%
set PYENVCFG=%VIRTUAL_ENV%\pyvenv.cfg

rem Hot-patching pyvenv.cfg
echo home = %ODMBASE%\python38> %PYENVCFG%
echo include-system-site-packages = false>> %PYENVCFG%

rem Hot-patching cv2 extension configs
echo BINARIES_PATHS = [r"%SBBIN%"] + BINARIES_PATHS> venv\Lib\site-packages\cv2\config.py
echo PYTHON_EXTENSIONS_PATHS = [r'%VIRTUAL_ENV%\lib\site-packages\cv2\python-3.8'] + PYTHON_EXTENSIONS_PATHS> venv\Lib\site-packages\cv2\config-3.8.py

if not defined PROMPT set PROMPT=$P$G

if defined _OLD_VIRTUAL_PROMPT set PROMPT=%_OLD_VIRTUAL_PROMPT%
if defined _OLD_VIRTUAL_PYTHONHOME set PYTHONHOME=%_OLD_VIRTUAL_PYTHONHOME%

set _OLD_VIRTUAL_PROMPT=%PROMPT%
set PROMPT=(venv) %PROMPT%

if defined PYTHONHOME set _OLD_VIRTUAL_PYTHONHOME=%PYTHONHOME%
set PYTHONHOME=

if defined _OLD_VIRTUAL_PATH set PATH=%_OLD_VIRTUAL_PATH%
if not defined _OLD_VIRTUAL_PATH set _OLD_VIRTUAL_PATH=%PATH%

set PATH=%VIRTUAL_ENV%\Scripts;%PATH%

:END
if defined _OLD_CODEPAGE (
    "%SystemRoot%\System32\chcp.com" %_OLD_CODEPAGE% > nul
    set _OLD_CODEPAGE=
)