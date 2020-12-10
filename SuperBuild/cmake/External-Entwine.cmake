set(_proj_name entwine)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  DEPENDS           pdal
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/connormanning/entwine/
  GIT_TAG           2.1.0
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DCMAKE_CXX_FLAGS=-isystem\ ${SB_SOURCE_DIR}/pdal
    -DADDITIONAL_LINK_DIRECTORIES_PATHS=${SB_INSTALL_DIR}/lib
    -DWITH_TESTS=OFF
	-DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
