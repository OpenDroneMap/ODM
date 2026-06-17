set(_proj_name mvstexturing)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

# Strip the hardcoded -march=native so the build falls back to the conda
# toolchain's portable baseline. Unlike OpenPointClass, this -march is not
# x86-guarded (it applies to aarch64 too), so we strip rather than substitute a
# fixed x86 -march. On macOS also bump the bundled Eigen 3.3.2 (which does not
# compile with clang 19) to 3.4.0.
set(MVSTEX_PATCH_COMMAND ${CMAKE_COMMAND} -DFILE=CMakeLists.txt -P ${SB_ROOT_DIR}/cmake/strip-march-native.cmake)
if(APPLE)
  list(APPEND MVSTEX_PATCH_COMMAND
    COMMAND ${CMAKE_COMMAND} -DFILE=elibs/CMakeLists.txt -P ${SB_ROOT_DIR}/cmake/patch-mvstex-eigen.cmake)
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           mve
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}/${_proj_name}
  GIT_REPOSITORY    https://github.com/OpenDroneMap/mvs-texturing
  GIT_TAG           c5a4d0c9a434553533c6e39d426e349fcfa5f48d
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  PATCH_COMMAND     ${MVSTEX_PATCH_COMMAND}
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DRESEARCH=OFF
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
    ${CONDA_CMAKE_ARGS} ${WIN32_CMAKE_ARGS}
    ${APPLE_CMAKE_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
