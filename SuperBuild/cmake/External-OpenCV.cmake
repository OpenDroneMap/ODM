set(_proj_name opencv)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if (WIN32)
  set(OCV_CMAKE_EXTRA_ARGS -DPYTHON3_NUMPY_INCLUDE_DIRS=${PYTHON_HOME}/lib/site-packages/numpy/core/include
                             -DPYTHON3_PACKAGES_PATH=${PYTHON_HOME}/lib/site-packages
                             -DPYTHON3_EXECUTABLE=${PYTHON_EXE_PATH}
                             -DWITH_MSMF=OFF
                             -DOPENCV_LIB_INSTALL_PATH=${SB_INSTALL_DIR}/lib
                             -DOPENCV_BIN_INSTALL_PATH=${SB_INSTALL_DIR}/bin)
elseif(APPLE)
  # macOS is unable to automatically detect our Python libs
  set(OCV_CMAKE_EXTRA_ARGS -DPYTHON3_NUMPY_INCLUDE_DIRS=${PYTHON_HOME}/lib/python3.8/site-packages/numpy/core/include
                           -DPYTHON3_PACKAGES_PATH=${PYTHON_HOME}/lib/python3.8/site-packages
                           -DPYTHON3_EXECUTABLE=${PYTHON_EXE_PATH}
                           -DPYTHON3_LIBRARIES=${HOMEBREW_INSTALL_PREFIX}/opt/python@3.8/Frameworks/Python.framework/Versions/3.8/lib/libpython3.8.dylib
                           -DPYTHON3_INCLUDE_DIR=${HOMEBREW_INSTALL_PREFIX}/opt/python@3.8/Frameworks/Python.framework/Versions/3.8/include/python3.8/
                           -DPYTHON3_INCLUDE_PATH=${HOMEBREW_INSTALL_PREFIX}/opt/python@3.8/Frameworks/Python.framework/Versions/3.8/include/python3.8/
                           -DPYTHON3INTERP_FOUND=ON
                           -DPYTHON3LIBS_FOUND=ON
                           -DPYTHON_DEFAULT_AVAILABLE=ON
                           -DPYTHON_DEFAULT_EXECUTABLE=${PYTHON_EXE_PATH}
                           -DPYTHON3_VERSION_MAJOR=3
                           -DPYTHON3_VERSION_MINOR=8
                           -DOPENCV_CONFIG_INSTALL_PATH=
                           -DOPENCV_PYTHON_INSTALL_PATH=${SB_INSTALL_DIR}/lib/python3.8/dist-packages
                           -DHAVE_opencv_python3=ON
                           -DOPENCV_PYTHON_SKIP_DETECTION=ON
                           -DOPENCV_LIB_INSTALL_PATH=${SB_INSTALL_DIR}/lib
                           -DOPENCV_BIN_INSTALL_PATH=${SB_INSTALL_DIR}/bin)
endif()

ExternalProject_Add(${_proj_name}
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/opencv/opencv/archive/4.5.0.zip
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DBUILD_opencv_core=ON
    -DBUILD_opencv_imgproc=ON
    -DBUILD_opencv_highgui=ON
    -DBUILD_opencv_video=ON
    -DBUILD_opencv_ml=ON
    -DBUILD_opencv_features2d=ON
    -DBUILD_opencv_calib3d=ON
    -DBUILD_opencv_contrib=ON
    -DBUILD_opencv_flann=ON
    -DBUILD_opencv_objdetect=ON
    -DBUILD_opencv_photo=ON
    -DBUILD_opencv_legacy=ON
    -DBUILD_opencv_python3=ON
    -DWITH_FFMPEG=OFF
    -DWITH_CUDA=OFF
    -DWITH_GTK=OFF
    -DWITH_VTK=OFF
    -DWITH_EIGEN=OFF
    -DWITH_OPENNI=OFF
    -DWITH_OPENEXR=OFF
    -DBUILD_EXAMPLES=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_PERF_TESTS=OFF
    -DBUILD_DOCS=OFF
    -DBUILD_opencv_apps=OFF
    -DBUILD_opencv_gpu=OFF
    -DBUILD_opencv_videostab=OFF
    -DBUILD_opencv_nonfree=OFF
    -DBUILD_opencv_stitching=OFF
    -DBUILD_opencv_world=OFF
    -DBUILD_opencv_superres=OFF
    -DBUILD_opencv_java=OFF
    -DBUILD_opencv_ocl=OFF
    -DBUILD_opencv_ts=OFF
    -DBUILD_opencv_xfeatures2d=ON
    -DOPENCV_ALLOCATOR_STATS_COUNTER_TYPE=int64_t
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
    ${WIN32_CMAKE_ARGS}
    ${OCV_CMAKE_EXTRA_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)