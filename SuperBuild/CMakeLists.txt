cmake_minimum_required(VERSION 3.1)

project(ODM-SuperBuild)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Setup SuperBuild root location
set(SB_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

################################
# Setup SuperBuild directories #
################################

# Setup location where source tar-balls are downloaded
set(SB_DOWNLOAD_DIR "${SB_ROOT_DIR}/download"
    CACHE PATH "Location where source tar-balls are (to be) downloaded.")
mark_as_advanced(SB_DOWNLOAD_DIR)

message(STATUS "SuperBuild files will be downloaded to: ${SB_DOWNLOAD_DIR}")


# Setup location where source tar-balls are located
set(SB_SOURCE_DIR "${SB_ROOT_DIR}/src"
    CACHE PATH "Location where source tar-balls are (will be).")
mark_as_advanced(SB_SOURCE_DIR)
set(SB_BUILD_DIR "${SB_ROOT_DIR}/build")

message(STATUS "SuperBuild source files will be extracted to: ${SB_SOURCE_DIR}")


# Setup location where source tar-balls are located
set(SB_INSTALL_DIR "${SB_ROOT_DIR}/install"
    CACHE PATH "Location where source tar-balls are (will be) installed.")
mark_as_advanced(SB_SOURCE_DIR)

message(STATUS "SuperBuild source files will be installed to: ${SB_INSTALL_DIR}")


# Setup location where binary files are located
set(SB_BINARY_DIR "${SB_ROOT_DIR}/build"
    CACHE PATH "Location where files are (will be) located.")
mark_as_advanced(SB_BINARY_DIR)

message(STATUS "SuperBuild binary files will be located to: ${SB_BINARY_DIR}")

if (WIN32)
  if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    message(FATAL_ERROR "CMAKE_TOOLCHAIN_FILE not set. You need to set it to the path of vcpkg.cmake")
  endif()
  get_filename_component(CMAKE_TOOLCHAIN_DIR ${CMAKE_TOOLCHAIN_FILE} DIRECTORY)
  get_filename_component(VCPKG_ROOT "${CMAKE_TOOLCHAIN_DIR}/../../" ABSOLUTE)
  set(WIN32_CMAKE_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  set(PYTHON_HOME "${SB_ROOT_DIR}/../venv")
  set(PYTHON_EXE_PATH "${PYTHON_HOME}/Scripts/python")

  # Use the GDAL version that comes with pip
  set(GDAL_ROOT "${PYTHON_HOME}/Lib/site-packages/osgeo")
  set(GDAL_LIBRARY "${GDAL_ROOT}/lib/gdal_i.lib")
  set(GDAL_INCLUDE_DIR "${GDAL_ROOT}/include/gdal")

  # Also download missing headers :/
  if (NOT EXISTS "${GDAL_INCLUDE_DIR}/ogrsf_frmts.h")
    file(DOWNLOAD "https://raw.githubusercontent.com/OSGeo/gdal/release/3.2/gdal/ogr/ogrsf_frmts/ogrsf_frmts.h" "${GDAL_INCLUDE_DIR}/ogrsf_frmts.h")
  endif()

  message("Copying VCPKG DLLs...")
  file(GLOB COPY_DLLS "${VCPKG_ROOT}/installed/x64-windows/bin/*.dll")
  file(COPY ${COPY_DLLS} DESTINATION "${SB_INSTALL_DIR}/bin")

  message("Copying CUDA DLLs...")
  file(GLOB CUDA_DLLS "$ENV{CUDA_PATH}/bin/cudart64*.dll")
  file(COPY ${CUDA_DLLS} DESTINATION "${SB_INSTALL_DIR}/bin")

  set(WIN32_GDAL_ARGS -DGDAL_FOUND=TRUE -DGDAL_LIBRARY=${GDAL_LIBRARY} -DGDAL_INCLUDE_DIR=${GDAL_INCLUDE_DIR})
else()
  set(PYTHON_EXE_PATH "/usr/bin/python3")
endif()

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${SB_ROOT_DIR}/cmake)

include(ExternalProject)
include(ExternalProject-Setup)

#########################################
# Download and install third party libs #
#########################################

# ---------------------------------------------------------------------------------------------
# Open Source Computer Vision (OpenCV)
#
set(ODM_OpenCV_Version 2.4.11)
option(ODM_BUILD_OpenCV "Force to build OpenCV library" OFF)

SETUP_EXTERNAL_PROJECT(OpenCV ${ODM_OpenCV_Version} ${ODM_BUILD_OpenCV})


# ---------------------------------------------------------------------------------------------
# Point Cloud Library (PCL)
#
set(ODM_PCL_Version 1.8.0)
option(ODM_BUILD_PCL "Force to build PCL library" OFF)

SETUP_EXTERNAL_PROJECT(PCL ${ODM_PCL_Version} ${ODM_BUILD_PCL})


# ---------------------------------------------------------------------------------------------
# Google Flags library (GFlags)
#
set(ODM_GFlags_Version 2.1.2)
option(ODM_BUILD_GFlags "Force to build GFlags library" OFF)

SETUP_EXTERNAL_PROJECT(GFlags ${ODM_GFlags_Version} ${ODM_BUILD_GFlags})


# ---------------------------------------------------------------------------------------------
# Ceres Solver
#
set(ODM_Ceres_Version 1.10.0)
option(ODM_BUILD_Ceres "Force to build Ceres library" OFF)

SETUP_EXTERNAL_PROJECT(Ceres ${ODM_Ceres_Version} ${ODM_BUILD_Ceres})


# ---------------------------------------------------------------------------------------------
# Hexer
#
SETUP_EXTERNAL_PROJECT(Hexer 1.4 ON)

# ---------------------------------------------------------------------------------------------
# Open Structure from Motion (OpenSfM)
#

set(custom_libs OpenSfM
				LASzip
				PDAL
				Untwine
				MvsTexturing
				OpenMVS
				FPCFilter
				PyPopsift
)

# Build entwine only on Linux
if (NOT WIN32)
    set(custom_libs ${custom_libs} Entwine)
endif()

externalproject_add(mve
    GIT_REPOSITORY  https://github.com/OpenDroneMap/mve.git
    GIT_TAG         262
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/mve
    CMAKE_ARGS      ${WIN32_CMAKE_ARGS}
    BUILD_IN_SOURCE 1
    INSTALL_COMMAND ""
)

foreach(lib ${custom_libs})
	SETUP_EXTERNAL_PROJECT_CUSTOM(${lib})
endforeach()

include(ProcessorCount)
ProcessorCount(nproc)

if (WIN32)
    set (POISSON_BUILD_CMD ${CMAKE_MAKE_PROGRAM} ${SB_SOURCE_DIR}/PoissonRecon/PoissonRecon.vcxproj /p:configuration=${CMAKE_BUILD_TYPE} /p:Platform=x64 /p:PlatformToolset=${CMAKE_VS_PLATFORM_TOOLSET} /p:WindowsTargetPlatformVersion=${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION})
    set (POISSON_BIN_PATH "x64/${CMAKE_BUILD_TYPE}/PoissonRecon.exe")
else()
    set (POISSON_BUILD_CMD  make -j${nproc} poissonrecon)
    set (POISSON_BIN_PATH "Linux/PoissonRecon")
endif()
externalproject_add(poissonrecon
    GIT_REPOSITORY    https://github.com/OpenDroneMap/PoissonRecon.git
    GIT_TAG           272
    PREFIX            ${SB_BINARY_DIR}/PoissonRecon
    SOURCE_DIR        ${SB_SOURCE_DIR}/PoissonRecon
    UPDATE_COMMAND    ""
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND    ${POISSON_BUILD_CMD}
    INSTALL_COMMAND  ${CMAKE_COMMAND} -E copy ${SB_SOURCE_DIR}/PoissonRecon/Bin/${POISSON_BIN_PATH} ${SB_INSTALL_DIR}/bin
)

externalproject_add(dem2mesh
    GIT_REPOSITORY    https://github.com/OpenDroneMap/dem2mesh.git
    GIT_TAG           master
    PREFIX            ${SB_BINARY_DIR}/dem2mesh
    SOURCE_DIR        ${SB_SOURCE_DIR}/dem2mesh
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${WIN32_GDAL_ARGS}
)

externalproject_add(dem2points
    GIT_REPOSITORY    https://github.com/OpenDroneMap/dem2points.git
    GIT_TAG           master
    PREFIX            ${SB_BINARY_DIR}/dem2points
    SOURCE_DIR        ${SB_SOURCE_DIR}/dem2points
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${WIN32_GDAL_ARGS}
)

externalproject_add(odm_orthophoto
    DEPENDS           pcl opencv 
    GIT_REPOSITORY    https://github.com/OpenDroneMap/odm_orthophoto.git
    GIT_TAG           main
    PREFIX            ${SB_BINARY_DIR}/odm_orthophoto
    SOURCE_DIR        ${SB_SOURCE_DIR}/odm_orthophoto
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${WIN32_CMAKE_ARGS} ${WIN32_GDAL_ARGS}
)

externalproject_add(lastools
    GIT_REPOSITORY    https://github.com/OpenDroneMap/LAStools.git
    GIT_TAG           250
    PREFIX            ${SB_BINARY_DIR}/lastools
    SOURCE_DIR        ${SB_SOURCE_DIR}/lastools
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
)
