set(_proj_name cmvs)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}/${_proj_name}
  URL               https://github.com/OpenDroneMap/CMVS-PMVS/archive/7224aecf050a1f789f5535c1744a5e1deaac4734.zip
  URL_MD5           24DF6898F824791FDA9D22E1465E2E43
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CONFIGURE_COMMAND cmake <SOURCE_DIR>/program
    -DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=${SB_INSTALL_DIR}/bin
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

