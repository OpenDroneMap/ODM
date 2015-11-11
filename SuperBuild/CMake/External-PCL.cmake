ExternalProject_Add(pcl
  URL https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz
  URL_MD5 02c72eb6760fcb1f2e359ad8871b9968
  BINARY_DIR ${SB_BUILD_DIR}
  INSTALL_DIR ${SB_INSTALL_PREFIX}
  DOWNLOAD_DIR ${SB_DOWNLOAD_LOCATION}
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
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_PREFIX}
)

set(PCL_DIR ${SB_INSTALL_PREFIX}/share)