set(_proj_name openmvs)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

externalproject_add(vcg
    GIT_REPOSITORY  https://github.com/OpenDroneMap/VCG.git
    GIT_TAG         285
    UPDATE_COMMAND  ""
    PATCH_COMMAND   ${CMAKE_COMMAND} -P ${SB_ROOT_DIR}/cmake/apply-patch.cmake ${SB_ROOT_DIR}/cmake/vcg-template-kw.patch
    SOURCE_DIR      ${SB_SOURCE_DIR}/vcg
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

externalproject_add(eigen34
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         7176ae16238ded7fb5ed30a7f5215825b3abd134
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/eigen34
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

SET(ARM64_CMAKE_ARGS "")

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
  SET(ARM64_CMAKE_ARGS -DOpenMVS_USE_SSE=OFF)
endif()

SET(GPU_CMAKE_ARGS "")
if(UNIX)
    # OpenMVS links libcuda, the NVIDIA driver library, which is absent wherever no
    # driver is installed. Fall back to the linker stub the CUDA toolkit ships: its
    # SONAME is libcuda.so.1 and the stubs directory holds no file of that name, so
    # the loader still resolves the real driver at runtime.
    find_library(SB_LIBCUDA cuda)
    if(NOT SB_LIBCUDA)
        foreach(_stubs "$ENV{CONDA_PREFIX}/lib/stubs" "/usr/local/cuda/lib64/stubs")
            if (EXISTS "${_stubs}/libcuda.so")
                SET(GPU_CMAKE_ARGS -DCMAKE_LIBRARY_PATH=${_stubs})
                break()
            endif()
        endforeach()
    endif()
endif()

if(WIN32)
  # On Windows systems without NVIDIA GPUs, OpenMVS will not launch
  # unless a CUDA DLL is available; we download a dummy DLL
  # generated with https://github.com/ykhwong/dummy-dll-generator that is
  # loaded UNLESS the real CUDA DLL is available, since it will
  # be loaded before our dummy DLL.
  file(DOWNLOAD "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/nvcuda_dummy.dll" "${SB_INSTALL_DIR}/bin/nvcuda.dll")
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           opencv vcg
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/OpenDroneMap/openMVS
  GIT_TAG           v2.4.0-odm.2
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DVCG_ROOT=${SB_SOURCE_DIR}/vcg
    -DCMAKE_INSTALL_PREFIX=${SB_INSTALL_DIR}
    -DOpenMVS_ENABLE_TESTS=OFF
    -DOpenMVS_MAX_CUDA_COMPATIBILITY=ON
    -DINSTALL_USE_SUBDIR=OFF
    -DOpenMVS_USE_CUDA=${SB_ENABLE_CUDA}
    -DCMAKE_CUDA_ARCHITECTURES=OFF
    ${GPU_CMAKE_ARGS}
    ${SB_EP_CMAKE_ARGS}
    ${CONDA_CMAKE_ARGS}
    ${ARM64_CMAKE_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
