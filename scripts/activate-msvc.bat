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

rem conda's vs20xx_compiler_vars.bat force the Visual Studio generator with a
rem platform/toolset. We build with Ninja + cl, so clear them: Ninja rejects
rem the platform/toolset specs and would otherwise error out.
set "CMAKE_GENERATOR=Ninja"
set "CMAKE_GENERATOR_PLATFORM="
set "CMAKE_GENERATOR_TOOLSET="

rem conda-forge keeps the CUDA CCCL/libcu++ headers (nv/target, cuda/std, thrust,
rem cub) under Library\include\targets\x64, not the include root. nvcc adds that
rem dir itself, but host cl.exe compiles (e.g. OpenMVS via legacy FindCUDA) don't,
rem so curand_kernel.h fails on '#include <nv/target>'. Put it on INCLUDE.
if exist "%CONDA_PREFIX%\Library\include\targets\x64\nv\target" set "INCLUDE=%CONDA_PREFIX%\Library\include\targets\x64;%INCLUDE%"
