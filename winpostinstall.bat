set ODMBASE=%~dp0
set VIRTUAL_ENV=%ODMBASE%venv
IF "%~1"=="" (set WRITABLE_VIRTUAL_ENV=%VIRTUAL_ENV%) ELSE (set WRITABLE_VIRTUAL_ENV=%~1)
mkdir "%WRITABLE_VIRTUAL_ENV%"

rem Hot-patching pyvenv.cfg
set PYENVCFG=%WRITABLE_VIRTUAL_ENV%\pyvenv.cfg
echo home = %VIRTUAL_ENV%\Scripts> "%PYENVCFG%"
echo include-system-site-packages = false>> "%PYENVCFG%"

rem Write python312._pth next to the embedded python.exe to lock down sys.path.
rem Without it, the "less-pth" embedded python merges registry PythonPath entries
rem from any system Python 3.12 (e.g. Anaconda), breaking ctypes/DLL loading.
set PTHFILE=%VIRTUAL_ENV%\Scripts\python312._pth
echo python312.zip> "%PTHFILE%"
echo .>> "%PTHFILE%"
echo ..\..>> "%PTHFILE%"
echo ..\Lib\site-packages>> "%PTHFILE%"
echo import site>> "%PTHFILE%"

rem Hot-patching cv2 extension configs
set SBBIN=%ODMBASE%SuperBuild\install\bin
set CV2=%WRITABLE_VIRTUAL_ENV%\Lib\site-packages\cv2
mkdir "%CV2%"
echo BINARIES_PATHS = [r"%SBBIN%"] + BINARIES_PATHS> "%CV2%\config.py"
echo PYTHON_EXTENSIONS_PATHS = [r'''%VIRTUAL_ENV%\lib\site-packages\cv2\python-3.12'''] + PYTHON_EXTENSIONS_PATHS> "%CV2%\config-3.12.py"

cls