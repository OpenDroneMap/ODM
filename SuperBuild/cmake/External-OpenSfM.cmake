set(_proj_name opensfm)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")
include(ProcessorCount)
ProcessorCount(nproc)

if(WIN32)
  set(OpenCV_DIR "${SB_INSTALL_DIR}/x64/vc16/lib")
  set(Python_PATH "${SB_INSTALL_DIR}/../../venv/Scripts/python")
  set(WIN32_CMAKE_EXTRA_ARGS -DCMAKE_CXX_FLAGS="-DM_PI=3.14159265358979323846")
  set(BUILD_CMD cmake --build "${SB_BUILD_DIR}/opensfm" --config "${CMAKE_BUILD_TYPE}")
else()
  set(OpenCV_DIR "${SB_INSTALL_DIR}/lib/cmake/opencv4")
  set(Python_PATH "/usr/bin/python3")
  set(BUILD_CMD make "-j${nproc}")
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           ceres opencv gflags
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/OpenDroneMap/OpenSfM/
  GIT_TAG           2410
  #--Update/Patch step----------
  UPDATE_COMMAND    git submodule update --init --recursive
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CONFIGURE_COMMAND cmake <SOURCE_DIR>/${_proj_name}/src
    -DCERES_ROOT_DIR=${SB_INSTALL_DIR}
    -DOpenCV_DIR=${OpenCV_DIR}
    -DADDITIONAL_INCLUDE_DIRS=${SB_INSTALL_DIR}/include
    -DOPENSFM_BUILD_TESTS=off
    -DPYTHON_EXECUTABLE=${Python_PATH}
    ${WIN32_CMAKE_ARGS}
    ${WIN32_CMAKE_EXTRA_ARGS}
  BUILD_COMMAND ${BUILD_CMD}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_COMMAND    ""
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
