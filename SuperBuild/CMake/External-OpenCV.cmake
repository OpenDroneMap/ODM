
ExternalProject_Add(opencv
  URL https://github.com/Itseez/opencv/archive/2.4.11.zip
  URL_MD5 b517e83489c709eee1d8be76b16976a7
  BINARY_DIR ${SB_BUILD_DIR}
  INSTALL_DIR ${SB_INSTALL_PREFIX}
  DOWNLOAD_DIR ${DOWNLOAD_LOCATION}
  CMAKE_ARGS
    -DBUILD_opencv_core=ON
    -DBUILD_opencv_imgproc=ON
    -DBUILD_opencv_highgui=ON
    -DWITH_CUDA=OFF
    -DWITH_GTK=OFF
    -DWITH_VTK=OFF
    -DWITH_EIGEN=OFF
    -DWITH_OPENNI=OFF
    -DBUILD_EXAMPLES=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_PERF_TESTS=OFF
    -DBUILD_DOCS=OFF
    -DBUILD_opencv_apps=OFF
    -DBUILD_opencv_flann=OFF
    -DBUILD_opencv_legacy=OFF
    -DBUILD_opencv_photo=OFF
    -DBUILD_opencv_video=OFF
    -DBUILD_opencv_calib3d=OFF
    -DBUILD_opencv_gpu=OFF
    -DBUILD_opencv_ml=OFF
    -DBUILD_opencv_python=OFF
    -DBUILD_opencv_videostab=OFF
    -DBUILD_opencv_contrib=OFF
    -DBUILD_opencv_nonfree=OFF
    -DBUILD_opencv_stitching=OFF
    -DBUILD_opencv_world=OFF
    -DBUILD_opencv_objdetect=OFF
    -DBUILD_opencv_superres=OFF
    -DBUILD_opencv_features2d=OFF
    -DBUILD_opencv_java=OFF
    -DBUILD_opencv_ocl=OFF
    -DBUILD_opencv_ts=OFF
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_PREFIX}
)

set(OPENCV_DIR ${SB_INSTALL_PREFIX}/share)