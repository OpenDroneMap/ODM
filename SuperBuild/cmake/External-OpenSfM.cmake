set(_proj_name opensfm)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")
include(ProcessorCount)
ProcessorCount(nproc)

set(EXTRA_INCLUDE_DIRS "")
if(WIN32)
  set(OpenCV_DIR "${SB_INSTALL_DIR}/x64/vc17/lib")
  set(BUILD_CMD ${CMAKE_COMMAND} --build . --config "${CMAKE_BUILD_TYPE}" --parallel ${nproc})
else()
  set(BUILD_CMD ${CMAKE_COMMAND} --build . --parallel ${nproc})
  if (APPLE)
    set(OpenCV_DIR "${SB_INSTALL_DIR}")
    set(EXTRA_INCLUDE_DIRS "${HOMEBREW_INSTALL_PREFIX}/include")
  else()
    if(DEFINED ENV{CONDA_PREFIX})
      set(OpenCV_DIR "$ENV{CONDA_PREFIX}/lib/cmake/opencv4")
      set(CERES_ROOT_DIR "$ENV{CONDA_PREFIX}")
      set(EXTRA_INCLUDE_DIRS "$ENV{CONDA_PREFIX}/include")
    else()
      set(OpenCV_DIR "${SB_INSTALL_DIR}/lib/cmake/opencv4")
      set(CERES_ROOT_DIR "${SB_INSTALL_DIR}")
    endif()
  endif()
endif()

set(OPENSFM_CERES_ROOT_DIR "${SB_INSTALL_DIR}")
set(OPENSFM_EXTRA_CXX_FLAGS "")
if(DEFINED ENV{CONDA_PREFIX})
  set(OPENSFM_CERES_ROOT_DIR "$ENV{CONDA_PREFIX}")
  # glog 0.6+ requires GLOG_USE_GLOG_EXPORT to be defined. This is normally
  # propagated via the glog::glog cmake target, but OpenSfM links Ceres with
  # the old-style ${CERES_LIBRARIES} variable and misses the transitive define.
  set(OPENSFM_EXTRA_CXX_FLAGS "$ENV{CXXFLAGS} -DGLOG_USE_GLOG_EXPORT -DGLOG_USE_GFLAGS")
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           ceres opencv gflags
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/OpenDroneMap/OpenSfM/
  GIT_TAG           c5328439465e6ace011f39077d1077d7b1cdd65d
  #--Update/Patch step----------
  UPDATE_COMMAND    git submodule update --init --recursive
  #--Configure step-------------
  SOURCE_DIR        ${SB_INSTALL_DIR}/bin/${_proj_name}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} <SOURCE_DIR>/${_proj_name}/src
    -DCERES_ROOT_DIR=${OPENSFM_CERES_ROOT_DIR}
    -DOpenCV_DIR=${OpenCV_DIR}
    -DADDITIONAL_INCLUDE_DIRS=${SB_INSTALL_DIR}/include
    -DYET_ADDITIONAL_INCLUDE_DIRS=${EXTRA_INCLUDE_DIRS}
    -DOPENSFM_BUILD_TESTS=off
    -DPYTHON_EXECUTABLE=${PYTHON_EXE_PATH}
    -DCMAKE_CXX_FLAGS=${OPENSFM_EXTRA_CXX_FLAGS}
    ${CONDA_CMAKE_ARGS} ${WIN32_CMAKE_ARGS}
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
