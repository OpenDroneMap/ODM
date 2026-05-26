@echo off

set "VSWHERE=%CONDA_PREFIX%\Library\bin\vswhere.exe"

set "VSINSTALLDIR="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
  set "VSINSTALLDIR=%%i"
)

if not defined VSINSTALLDIR (
  echo ERROR: No Visual Studio installation with C++ tools found.
  echo Install VS 2022 or Build Tools with "Desktop development with C++" and Windows SDK.
  exit /b 1
)

call "%VSINSTALLDIR%\Common7\Tools\VsDevCmd.bat" -arch=amd64

rem Conda compiler-rt sets clang -Wl flags that break MSVC link.exe
set "LDFLAGS="
set "CXXFLAGS="
set "CMAKE_EXE_LINKER_FLAGS="
set "CMAKE_SHARED_LINKER_FLAGS="
set "CMAKE_MODULE_LINKER_FLAGS="
