set(_proj_name ecto)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  DEPENDS           catkin
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}/${_proj_name}
  URL               https://github.com/plasmodic/ecto/archive/c6178ed0102a66cebf503a4213c27b0f60cfca69.zip
  URL_MD5           A5C4757B656D536D3E3CC1DC240EC158
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DBUILD_DOC=OFF
    -DBUILD_SAMPLES=OFF
    -DCATKIN_ENABLE_TESTING=OFF
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
