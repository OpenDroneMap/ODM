set(_proj_name openmvs)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

externalproject_add(vcg
    GIT_REPOSITORY  https://github.com/cdcseacave/VCG.git
    GIT_TAG         master
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/vcg
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

externalproject_add(eigen34
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         3.4
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/eigen34
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

SET(ARM64_CMAKE_ARGS "")
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
  SET(ARM64_CMAKE_ARGS -DOpenMVS_USE_SSE=OFF)
endif()

SET(GPU_CMAKE_ARGS "")
if(UNIX)
    if (EXISTS "/usr/local/cuda/lib64/stubs")
        SET(GPU_CMAKE_ARGS -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs)
    endif()
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           ceres opencv vcg eigen34
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/OpenDroneMap/openMVS
  GIT_TAG           266
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DOpenCV_DIR=${SB_INSTALL_DIR}/lib/cmake/opencv4
    -DVCG_ROOT=${SB_SOURCE_DIR}/vcg
    -DEIGEN3_INCLUDE_DIR=${SB_SOURCE_DIR}/eigen34/
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=${SB_INSTALL_DIR}
    ${GPU_CMAKE_ARGS}
    ${WIN32_CMAKE_ARGS}
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
