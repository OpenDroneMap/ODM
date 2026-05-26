set(_proj_name opensfm)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")
include(ProcessorCount)
ProcessorCount(nproc)

set(EXTRA_INCLUDE_DIRS "")
if(WIN32)
  set(BUILD_CMD ${CMAKE_COMMAND} --build "${SB_BUILD_DIR}/opensfm" --config "${CMAKE_BUILD_TYPE}")
  if(DEFINED ENV{CONDA_PREFIX})
    # Match conda-forge OpenCV layout on win-64 (same tree CMake reports at configure time).
    set(OpenCV_DIR "$ENV{CONDA_PREFIX}/Library/cmake/x64/vc17/lib")
    list(APPEND EXTRA_INCLUDE_DIRS "$ENV{CONDA_PREFIX}/Library/include")
  else()
    set(OpenCV_DIR "${SB_INSTALL_DIR}/x64/vc17/lib")
  endif()
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

set(OPENSFM_CONFIGURE_ARGS "")
if(WIN32 AND DEFINED ENV{CONDA_PREFIX})
  list(APPEND OPENSFM_CONFIGURE_ARGS
    "-DCMAKE_EXE_LINKER_FLAGS="
    "-DCMAKE_SHARED_LINKER_FLAGS="
    "-DCMAKE_MODULE_LINKER_FLAGS=")
endif()

set(OPENSFM_PATCH_COMMAND "${CMAKE_COMMAND}" -E true)
if(WIN32)
  set(OPENSFM_PATCH_COMMAND git apply ${CMAKE_MODULE_PATH}/opensfm-windows-ceres-glog.patch)
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
  PATCH_COMMAND     ${OPENSFM_PATCH_COMMAND}
  #--Configure step-------------
  SOURCE_DIR        ${SB_INSTALL_DIR}/bin/${_proj_name}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} <SOURCE_DIR>/${_proj_name}/src
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCERES_ROOT_DIR=${OPENSFM_CERES_ROOT_DIR}
    -DOpenCV_DIR=${OpenCV_DIR}
    -DADDITIONAL_INCLUDE_DIRS=${SB_INSTALL_DIR}/include
    -DYET_ADDITIONAL_INCLUDE_DIRS=${EXTRA_INCLUDE_DIRS}
    -DOPENSFM_BUILD_TESTS=off
    -DPYTHON_EXECUTABLE=${PYTHON_EXE_PATH}
<<<<<<< HEAD
    -DCMAKE_CXX_FLAGS=${OPENSFM_EXTRA_CXX_FLAGS}
=======
    ${OPENSFM_CONFIGURE_ARGS}
>>>>>>> 9aeff403 (feat: Native windows pixi build)
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
