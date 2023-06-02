set(_proj_name exiftool)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if (WIN32)
ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/exiftool.zip
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  UPDATE_COMMAND    ""
  CONFIGURE_COMMAND ""
  BUILD_IN_SOURCE 1
  BUILD_COMMAND     ""
  INSTALL_COMMAND  ${CMAKE_COMMAND} -E copy ${SB_SOURCE_DIR}/${_proj_name}/exiftool.exe ${SB_INSTALL_DIR}/bin
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
else()
externalproject_add(${_proj_name}
    PREFIX            ${_SB_BINARY_DIR}
    TMP_DIR           ${_SB_BINARY_DIR}/tmp
    STAMP_DIR         ${_SB_BINARY_DIR}/stamp
    SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
    #--Download step--------------
    DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
    URL               https://github.com/exiftool/exiftool/archive/refs/tags/12.62.zip
    UPDATE_COMMAND    ""
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND    perl Makefile.PL PREFIX=${SB_INSTALL_DIR} LIB=${SB_INSTALL_DIR}/bin/lib
    INSTALL_COMMAND  make install && rm -fr ${SB_INSTALL_DIR}/man
)
endif()