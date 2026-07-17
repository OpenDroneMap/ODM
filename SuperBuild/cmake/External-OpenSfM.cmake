set(_proj_name opensfm)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")
include(ProcessorCount)
ProcessorCount(nproc)

set(EXTRA_INCLUDE_DIRS "")
set(OPENSFM_ADDITIONAL_INCLUDE_DIRS "")
set(OPENSFM_EXTRA_LINKER_FLAGS "")
if(WIN32)
  set(BUILD_CMD ${CMAKE_COMMAND} --build "${SB_BUILD_DIR}/opensfm" --config "${CMAKE_BUILD_TYPE}")
  # Keep OpenSfM on conda headers to avoid mixing with SuperBuild-installed headers.
  set(OPENSFM_ADDITIONAL_INCLUDE_DIRS "$ENV{CONDA_PREFIX}/Library/include")
  # Match conda-forge OpenCV layout on win-64 (same tree CMake reports at configure time).
  list(APPEND EXTRA_INCLUDE_DIRS "$ENV{CONDA_PREFIX}/Library/include")
  # OpenSfM links Ceres via the old-style ${CERES_LIBRARIES} variable, which on
  # Windows is just ceres.lib and drops glog/gflags. OpenSfM's own objects call
  # glog directly, so MSVC fails with unresolved __imp_ (dllimport) glog symbols.
  # Force-link glog + gflags into every target. Linux gets these transitively.
  set(OPENSFM_EXTRA_LINKER_FLAGS "$ENV{CONDA_PREFIX}/Library/lib/glog.lib $ENV{CONDA_PREFIX}/Library/lib/gflags.lib")
else()
  set(OPENSFM_ADDITIONAL_INCLUDE_DIRS "${SB_INSTALL_DIR}/include")
  set(BUILD_CMD ${CMAKE_COMMAND} --build . --parallel ${nproc})
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
  # TEMPORARY: fork on top of https://github.com/OpenDroneMap/OpenSfM/pull/46 for
  # for a small fix. Once all merged, use OpenDroneMap source.
  GIT_REPOSITORY    https://github.com/spwoodcock/OpenSfM/
  GIT_TAG           bb50bbb7b859faf201e4583c2d721a71785a4604
  #--Update/Patch step----------
  UPDATE_COMMAND    git submodule update --init --recursive
  #--Configure step-------------
  SOURCE_DIR        ${SB_INSTALL_DIR}/bin/${_proj_name}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} <SOURCE_DIR>/${_proj_name}/src
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCERES_ROOT_DIR=${OPENSFM_CERES_ROOT_DIR}
    -DOpenCV_DIR=${OpenCV_DIR}
    -DADDITIONAL_INCLUDE_DIRS=${OPENSFM_ADDITIONAL_INCLUDE_DIRS}
    -DYET_ADDITIONAL_INCLUDE_DIRS=${EXTRA_INCLUDE_DIRS}
    -DOPENSFM_BUILD_TESTS=off
    -DPYTHON_EXECUTABLE=${PYTHON_EXE_PATH}
    -DCMAKE_CXX_FLAGS=${OPENSFM_EXTRA_CXX_FLAGS}
    "-DCMAKE_MODULE_LINKER_FLAGS=${OPENSFM_EXTRA_LINKER_FLAGS}"
    "-DCMAKE_SHARED_LINKER_FLAGS=${OPENSFM_EXTRA_LINKER_FLAGS}"
    "-DCMAKE_EXE_LINKER_FLAGS=${OPENSFM_EXTRA_LINKER_FLAGS}"
    ${CONDA_CMAKE_ARGS}
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
