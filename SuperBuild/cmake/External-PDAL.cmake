set(_proj_name pdal)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if (WIN32)
set(LASZIP_LIB "${SB_INSTALL_DIR}/lib/laszip.lib")
else()
set(LASZIP_LIB "${SB_INSTALL_DIR}/lib/liblaszip.so")
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           hexer laszip
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/PDAL/PDAL/archive/refs/tags/2.3RC1.zip
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DBUILD_PGPOINTCLOUD_TESTS=OFF
    -DBUILD_PLUGIN_PGPOINTCLOUD=OFF
    -DBUILD_PLUGIN_CPD=OFF
    -DBUILD_PLUGIN_GREYHOUND=OFF
    -DBUILD_PLUGIN_HEXBIN=ON
    -DBUILD_PLUGIN_ICEBRIDGE=OFF
    -DBUILD_PLUGIN_MRSID=OFF
    -DBUILD_PLUGIN_NITF=OFF
    -DBUILD_PLUGIN_OCI=OFF
    -DBUILD_PLUGIN_P2G=OFF
    -DBUILD_PLUGIN_SQLITE=OFF
    -DBUILD_PLUGIN_RIVLIB=OFF
    -DBUILD_PLUGIN_PYTHON=OFF
    -DWITH_ZSTD=OFF
    -DENABLE_CTEST=OFF
    -DWITH_APPS=ON
    -DWITH_LAZPERF=OFF
    -DWITH_GEOTIFF=ON
    -DWITH_LASZIP=ON
    -DLASZIP_FOUND=TRUE
    -DLASZIP_LIBRARIES=${LASZIP_LIB}
    -DLASZIP_VERSION=3.1.1
    -DLASZIP_INCLUDE_DIR=${SB_INSTALL_DIR}/include
    -DLASZIP_LIBRARY=${LASZIP_LIB}
    -DWITH_TESTS=OFF
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
    ${WIN32_CMAKE_ARGS}
    ${WIN32_GDAL_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
