set(_proj_name openpointclass)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  DEPENDS           pdal eigen34
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/uav4geo/OpenPointClass
  GIT_TAG           dd6a560a1d43cb709f7b220b19a436e25a889e3e
  #--Update/Patch step----------
  # OpenPointClass hardcodes -march=native; rather than patch it out we use the
  # project's own PORTABLE_BUILD option (CMAKE_ARGS below), which selects its
  # x86-guarded -march=nehalem branch (the x86-64-v2 baseline ODM targets).
  UPDATE_COMMAND    ""
  PATCH_COMMAND     ${CMAKE_COMMAND} -DFILE=CMakeLists.txt -DBUILD_TYPE=${CMAKE_BUILD_TYPE} -P ${SB_ROOT_DIR}/cmake/set-lightgbm-build-type.cmake
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DPORTABLE_BUILD=ON
    -DWITH_GBT=ON
    -DBUILD_PCTRAIN=OFF
    -DEIGEN3_INCLUDE_DIR=${SB_SOURCE_DIR}/eigen34/
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
    ${SB_EP_CMAKE_ARGS}
    ${CONDA_CMAKE_ARGS} ${WIN32_CMAKE_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
