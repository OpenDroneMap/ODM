set(_proj_name opensfm)
set(_SB_BINARY_DIR "${CMAKE_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  DEPENDS           ceres opencv
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/mapillary/OpenSfM/archive/odm-1.zip
  URL_MD5           5261d2df9af2b29a3bfde0b29421d108
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CONFIGURE_COMMAND cmake <SOURCE_DIR>/${_proj_name}/src
    -DCERES_ROOT_DIR=${SB_INSTALL_DIR} 
    -DOpenCV_DIR=${SB_INSTALL_DIR}/share/OpenCV
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_COMMAND    ""
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)

