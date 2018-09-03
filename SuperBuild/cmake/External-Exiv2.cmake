set(_proj_name exiv2lib)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}/${_proj_name}
  URL               http://www.exiv2.org/builds/exiv2-0.26-trunk.tar.gz
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${SB_INSTALL_DIR}
    -DCMAKE_INSTALL_LIBDIR=lib
  BUILD_IN_SOURCE   ON
  BUILD_COMMAND     make exiv2lib
  INSTALL_DIR       ${SB_INSTALL_DIR}
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)