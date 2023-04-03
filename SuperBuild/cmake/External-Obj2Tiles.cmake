set(_proj_name obj2tiles)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

set(OBJ2TILES_VERSION v1.0.12)
set(OBJ2TILES_EXT "")

set(OBJ2TILES_ARCH "Linux64")
if (WIN32)
    set(OBJ2TILES_ARCH "Win64")
    set(OBJ2TILES_EXT ".exe")
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(OBJ2TILES_ARCH "LinuxArm")
elseif(APPLE)
    set(OBJ2TILES_ARCH "Osx64")
endif()


ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL           https://github.com/OpenDroneMap/Obj2Tiles/releases/download/${OBJ2TILES_VERSION}/Obj2Tiles-${OBJ2TILES_ARCH}.zip
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  UPDATE_COMMAND    ""
  CONFIGURE_COMMAND ""
  BUILD_IN_SOURCE 1
  BUILD_COMMAND     ""
  INSTALL_COMMAND  ${CMAKE_COMMAND} -E copy ${SB_SOURCE_DIR}/${_proj_name}/Obj2Tiles${OBJ2TILES_EXT} ${SB_INSTALL_DIR}/bin
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)