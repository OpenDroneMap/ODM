@echo off

rem This file is UTF-8 encoded, so we need to update the current code page while executing it
for /f "tokens=2 delims=:." %%a in ('"%SystemRoot%\System32\chcp.com"') do (
    set _OLD_CODEPAGE=%%a
)
if defined _OLD_CODEPAGE (
    "%SystemRoot%\System32\chcp.com" 65001 > nul
)

rem ODMBASE is the install directory (trailing backslash). The bundled conda
rem runtime environment and the native SuperBuild binaries are both located
rem relative to it, so the package relocates to any install path without a
rem prefix fix-up step and without requiring pixi on the target machine.
set ODMBASE=%~dp0
set CONDA_PREFIX=%ODMBASE%.pixi\envs\prod

set SBBIN=%ODMBASE%SuperBuild\install\bin
set OSFMBASE=%SBBIN%\opensfm\bin
set PDAL_DRIVER_PATH=%SBBIN%

if not defined ODMDONTSETPYCACHE set PYTHONPYCACHEPREFIX=%PROGRAMDATA%\ODM\pycache
set PYTHONUTF8=1

rem Conda environment search path (mirrors `conda activate` on Windows), with the
rem SuperBuild binaries appended so opensfm/openmvs/etc. and their DLLs resolve.
set PATH=%CONDA_PREFIX%;%CONDA_PREFIX%\Library\mingw-w64\bin;%CONDA_PREFIX%\Library\usr\bin;%CONDA_PREFIX%\Library\bin;%CONDA_PREFIX%\Scripts;%CONDA_PREFIX%\bin;%SBBIN%;%OSFMBASE%;%PATH%

rem Run the conda activation scripts shipped by gdal/proj/pdal/etc. They set
rem GDAL_DATA, PROJ_DATA, etc. relative to %CONDA_PREFIX%, so they relocate too.
if exist "%CONDA_PREFIX%\etc\conda\activate.d" (
    for %%f in ("%CONDA_PREFIX%\etc\conda\activate.d\*.bat") do call "%%f"
)

if not defined PROMPT set PROMPT=$P$G
set PROMPT=(ODM) %PROMPT%

:END
if defined _OLD_CODEPAGE (
    "%SystemRoot%\System32\chcp.com" %_OLD_CODEPAGE% > nul
    set _OLD_CODEPAGE=
)
