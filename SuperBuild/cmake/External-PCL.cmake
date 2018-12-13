set(_proj_name pcl)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}

  # PCL 1.8 + Fix for loading large point clouds https://github.com/OpenDroneMap/pcl/commit/924ab1137fbfa3004f222fb0834e3d66881ec057
  URL               https://github.com/OpenDroneMap/pcl/archive/master.zip
  
  #-- TODO: Use PCL 1.9.1 when we upgrade to a newer version of Ubuntu. Currently
  #-- it's troublesome to compile due to the older version of Boost shipping with 16.04.
  #-- URL               https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
  
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DBUILD_features=OFF
    -DBUILD_filters=OFF
    -DBUILD_geometry=OFF
    -DBUILD_keypoints=OFF
    -DBUILD_outofcore=OFF
    -DBUILD_people=OFF
    -DBUILD_recognition=OFF
    -DBUILD_registration=OFF
    -DBUILD_sample_consensus=OFF
    -DBUILD_segmentation=OFF
    -DBUILD_features=OFF
    -DBUILD_surface_on_nurbs=OFF
    -DBUILD_tools=OFF
    -DBUILD_tracking=OFF
    -DBUILD_visualization=OFF
    -DWITH_QT=OFF
    -DBUILD_OPENNI=OFF
    -DBUILD_OPENNI2=OFF
    -DWITH_OPENNI=OFF
    -DWITH_OPENNI2=OFF
    -DWITH_FZAPI=OFF
    -DWITH_LIBUSB=OFF
    -DWITH_PCAP=OFF
    -DWITH_PXCAPI=OFF
    -DCMAKE_BUILD_TYPE=Release
    -DPCL_VERBOSITY_LEVEL=Error
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