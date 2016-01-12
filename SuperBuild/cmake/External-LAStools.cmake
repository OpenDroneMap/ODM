set(_proj_name las-tools)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               http://lastools.org/download/LAStools.zip
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CONFIGURE_COMMAND ""
  WORKING_DIRECTORY "${SB_SOURCE_DIR}/${_proj_name}"
  #--Build step-----------------
  BUILD_IN_SOURCE 1
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  INSTALL_COMMAND ""
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
