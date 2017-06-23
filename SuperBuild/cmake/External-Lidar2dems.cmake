set(_proj_name lidar2dems)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}/${_proj_name}
  URL               https://github.com/pierotofy/lidar2dems/archive/master.zip
  URL_MD5           cece43dcf578ad47f169ee9078e17e8c
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CONFIGURE_COMMAND ""
  #--Build step-----------------
  BUILD_COMMAND     ""
  #--Install step---------------
  INSTALL_COMMAND   "${SB_SOURCE_DIR}/${_proj_name}/install.sh" 
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
