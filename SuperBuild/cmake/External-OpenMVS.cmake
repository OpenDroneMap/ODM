set(_proj_name openmvs)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

externalproject_add(vcg
    GIT_REPOSITORY  https://github.com/OpenDroneMap/VCG.git
    GIT_TAG         285
    UPDATE_COMMAND  ""
    PATCH_COMMAND   ${CMAKE_COMMAND} -P ${SB_ROOT_DIR}/cmake/apply-patch.cmake ${SB_ROOT_DIR}/cmake/vcg-template-kw.patch
    SOURCE_DIR      ${SB_SOURCE_DIR}/vcg
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

externalproject_add(eigen34
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         7176ae16238ded7fb5ed30a7f5215825b3abd134
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/eigen34
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

SET(ARM64_CMAKE_ARGS "")

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
  SET(ARM64_CMAKE_ARGS -DOpenMVS_USE_SSE=OFF)
endif()

SET(GPU_CMAKE_ARGS "")
if(UNIX)
    if (EXISTS "/usr/local/cuda/lib64/stubs")
        SET(GPU_CMAKE_ARGS -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs)
    endif()
endif()

if(WIN32)
  # On Windows systems without NVIDIA GPUs, OpenMVS will not launch
  # unless a CUDA DLL is available; we download a dummy DLL
  # generated with https://github.com/ykhwong/dummy-dll-generator that is
  # loaded UNLESS the real CUDA DLL is available, since it will
  # be loaded before our dummy DLL.
  file(DOWNLOAD "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/nvcuda_dummy.dll" "${SB_INSTALL_DIR}/bin/nvcuda.dll")
endif()

if(DEFINED ENV{CONDA_PREFIX})
  set(OPENMVS_OPENCV_DIR "$ENV{CONDA_PREFIX}/lib/cmake/opencv4")
else()
  set(OPENMVS_OPENCV_DIR "${SB_INSTALL_DIR}/lib/cmake/opencv4")
endif()

set(OPENMVS_WIN_CONDA_ARGS "")
if(WIN32 AND DEFINED ENV{CONDA_PREFIX})
  # conda-forge ships shared Boost (.lib import libs + .dll). Legacy FindBoost in OpenMVS
  # otherwise records .dll paths and omits BOOST_*_DYN_LINK.
  # boost-cpp links iostreams against conda zlib but does not ship boost_zlib.lib; MSVC
  # auto-link still requests it — see https://github.com/conda-forge/boost-cpp-feedstock/issues/65
  # and conda-forge/imp-feedstock recipe/bld.bat (BOOST_ZLIB_BINARY=kernel32).
  set(_conda_lib "$ENV{CONDA_PREFIX}/Library/lib")
  list(APPEND OPENMVS_WIN_CONDA_ARGS
    ${SB_EP_CMAKE_ARGS}
    -DBoost_USE_STATIC_LIBS=OFF
    -DBoost_USE_RELEASE_LIBS=ON
    -DCGAL_Boost_USE_STATIC_LIBS=OFF
    "-DBoost_IOSTREAMS_LIBRARY_RELEASE=${_conda_lib}/boost_iostreams.lib"
    "-DBoost_PROGRAM_OPTIONS_LIBRARY_RELEASE=${_conda_lib}/boost_program_options.lib"
    "-DBoost_SERIALIZATION_LIBRARY_RELEASE=${_conda_lib}/boost_serialization.lib"
    "-DBoost_SYSTEM_LIBRARY_RELEASE=${_conda_lib}/boost_system.lib"
    "-DCMAKE_CXX_FLAGS=/DBOOST_IOSTREAMS_DYN_LINK /DBOOST_PROGRAM_OPTIONS_DYN_LINK /DBOOST_SYSTEM_DYN_LINK /DBOOST_SERIALIZATION_DYN_LINK /DBOOST_ZLIB_BINARY=kernel32")
  unset(_conda_lib)
endif()

# On macOS the conda-forge clang 19 toolchain rejects CGAL 5.6.1's safe-bool
# operators on the half-edge iterators (CGAL #8313). conda-forge has no 5.6.2,
# so patch the header in the active environment before building OpenMVS.
if(APPLE)
  set(OPENMVS_PATCH_COMMAND ${CMAKE_COMMAND} -DFILE=$ENV{CONDA_PREFIX}/include/CGAL/boost/graph/iterator.h -P ${SB_ROOT_DIR}/cmake/patch-cgal-iterator.cmake)
else()
  set(OPENMVS_PATCH_COMMAND ${CMAKE_COMMAND} -E true)
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           ceres opencv vcg eigen34
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/OpenDroneMap/openMVS
  GIT_TAG           355
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  PATCH_COMMAND     ${OPENMVS_PATCH_COMMAND}
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DOpenCV_DIR=${OPENMVS_OPENCV_DIR}
    -DVCG_ROOT=${SB_SOURCE_DIR}/vcg
    -DEIGEN3_INCLUDE_DIR=${SB_SOURCE_DIR}/eigen34/
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${SB_INSTALL_DIR}
    -DOpenMVS_ENABLE_TESTS=OFF
    -DOpenMVS_MAX_CUDA_COMPATIBILITY=ON
    ${GPU_CMAKE_ARGS}
    ${CONDA_CMAKE_ARGS}
    ${OPENMVS_WIN_CONDA_ARGS}
    ${ARM64_CMAKE_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
