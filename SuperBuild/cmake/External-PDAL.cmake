set(_proj_name pdal)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/PDAL/PDAL/archive/d242c6704aafe85fd49fda11adae63d07ce11b76.zip 
  URL_MD5           14a7319e1f8483808eb93732cfa6511a
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -BUILD_PGPOINTCLOUD_TESTS=OFF
    -BUILD_PLUGIN_PCL=ON
    -BUILD_PLUGIN_PGPOINTCLOUD=ON
    -DBUILD_PLUGIN_CPD=OFF
	-DBUILD_PLUGIN_GREYHOUND=OFF
	-DBUILD_PLUGIN_HEXBIN=OFF
	-DBUILD_PLUGIN_ICEBRIDGE=OFF
	-DBUILD_PLUGIN_MRSID=OFF
	-DBUILD_PLUGIN_NITF=OFF
	-DBUILD_PLUGIN_OCI=OFF
	-DBUILD_PLUGIN_P2G=OFF
	-DBUILD_PLUGIN_SQLITE=OFF
	-DBUILD_PLUGIN_RIVLIB=OFF
	-DBUILD_PLUGIN_PYTHON=OFF
	-DENABLE_CTEST=OFF
	-DWITH_APPS=ON
	-DWITH_LAZPERF=OFF
	-DWITH_GEOTIFF=ON
	-DWITH_LASZIP=ON
	-DWITH_TESTS=OFF
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
