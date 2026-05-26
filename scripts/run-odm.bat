@echo off
setlocal EnableExtensions

set "ROOT=%~dp0.."
set "PATH=%CONDA_PREFIX%;%CONDA_PREFIX%\Library\bin;%CONDA_PREFIX%\Scripts%;%ROOT%\SuperBuild\install\bin;%PATH%"
set "PDAL_DRIVER_PATH=%ROOT%\SuperBuild\install\bin"
set "PROJ_LIB=%CONDA_PREFIX%\Library\share\proj"
set "GDAL_DATA=%CONDA_PREFIX%\Library\share\gdal"

for /f "usebackq delims=" %%i in (`python -c "import opendm.context, os; print(os.pathsep.join(opendm.context.python_packages_paths))"`) do set "PYTHONPATH=%%i;%PYTHONPATH%"

cd /d "%ROOT%"
python -X utf8 run.py %*

endlocal
