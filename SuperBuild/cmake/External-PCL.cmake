set(_proj_name pcl)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL           https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.11.1.zip
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
    -DWITH_OPENGL=OFF
    -DWITH_VTK=OFF
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
    -DPCL_BUILD_WITH_FLANN_DYNAMIC_LINKING_WIN32=ON
    ${WIN32_CMAKE_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)