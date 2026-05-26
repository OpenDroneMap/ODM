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
  UPDATE_COMMAND    ""
<<<<<<< HEAD
  PATCH_COMMAND     ${CMAKE_COMMAND} -DFILE=CMakeLists.txt -P ${SB_ROOT_DIR}/cmake/strip-march-native.cmake
=======
  PATCH_COMMAND     git apply ${CMAKE_MODULE_PATH}/openpointclass-lightgbm.patch
>>>>>>> 9aeff403 (feat: Native windows pixi build)
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
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
