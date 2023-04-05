set ODMBASE=%~dp0
set VIRTUAL_ENV=%ODMBASE%venv
set PYENVCFG=%VIRTUAL_ENV%\pyvenv.cfg
set SBBIN=%ODMBASE%SuperBuild\install\bin

rem Hot-patching pyvenv.cfg
echo home = %ODMBASE%venv\Scripts> "%PYENVCFG%"
echo include-system-site-packages = false>> "%PYENVCFG%"

rem Hot-patching cv2 extension configs
echo BINARIES_PATHS = [r"%SBBIN%"] + BINARIES_PATHS> venv\Lib\site-packages\cv2\config.py
echo PYTHON_EXTENSIONS_PATHS = [r'''%VIRTUAL_ENV%\lib\site-packages\cv2\python-3.8'''] + PYTHON_EXTENSIONS_PATHS> venv\Lib\site-packages\cv2\config-3.8.py

cls