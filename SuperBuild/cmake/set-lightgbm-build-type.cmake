# Forwards the SuperBuild CMAKE_BUILD_TYPE into OpenPointClass's nested LightGBM
# ExternalProject. LightGBM defaults to Debug when no build type is given, which
# on MSVC produces a /MDd (debug CRT) lib that fails to link against the Release
# (/MD) OpenPointClass objects (LNK2038 RuntimeLibrary / _ITERATOR_DEBUG_LEVEL).
# The REPLACE is a no-op once applied, so the step is idempotent.
# Usage: cmake -DFILE=<path> -DBUILD_TYPE=<cfg> -P set-lightgbm-build-type.cmake
file(READ "${FILE}" _content)
string(REPLACE
  "CMAKE_ARGS -DBUILD_STATIC_LIB=ON"
  "CMAKE_ARGS -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DBUILD_STATIC_LIB=ON"
  _content "${_content}")
file(WRITE "${FILE}" "${_content}")
