cmake_minimum_required(VERSION 3.1)

project(ODM-SuperBuild)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Setup SuperBuild root location
set(SB_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${SB_ROOT_DIR}/cmake)

include(ExternalProject)
include(ExternalProject-Setup)

option(ODM_BUILD_SLAM "Build SLAM module" OFF)


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
# Open Geometric Vision (OpenGV)
# Open Structure from Motion (OpenSfM)
#

set(custom_libs OpenGV
				OpenSfM
				LASzip
                Zstd
				PDAL
                Entwine
				MvsTexturing
)

# Dependencies of the SLAM module
if(ODM_BUILD_SLAM)
	list(APPEND custom_libs
				Pangolin
				ORB_SLAM2)
endif()

foreach(lib ${custom_libs})
	SETUP_EXTERNAL_PROJECT_CUSTOM(${lib})
endforeach()

## Add mve Build

externalproject_add(mve
    GIT_REPOSITORY  https://github.com/OpenDroneMap/mve.git
    GIT_TAG         099
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/elibs/mve
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND   make
    INSTALL_COMMAND ""
)

externalproject_add(poissonrecon
    GIT_REPOSITORY    https://github.com/mkazhdan/PoissonRecon.git
    GIT_TAG           ce5005ae3094d902d551a65a8b3131e06f45e7cf
    SOURCE_DIR        ${SB_SOURCE_DIR}/PoissonRecon
    UPDATE_COMMAND    ""
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND     make poissonrecon
    INSTALL_COMMAND   ""
)

externalproject_add(dem2mesh
    GIT_REPOSITORY    https://github.com/OpenDroneMap/dem2mesh.git
    GIT_TAG           master
    SOURCE_DIR        ${SB_SOURCE_DIR}/dem2mesh
    UPDATE_COMMAND    ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND     make
    INSTALL_COMMAND   ""
)

externalproject_add(dem2points
    GIT_REPOSITORY    https://github.com/OpenDroneMap/dem2points.git
    GIT_TAG           master
    SOURCE_DIR        ${SB_SOURCE_DIR}/dem2points
    UPDATE_COMMAND    ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND     make
    INSTALL_COMMAND   ""
)

externalproject_add(lastools
    GIT_REPOSITORY    https://github.com/LAStools/LAStools.git
    GIT_TAG           2ef44281645999ec7217facec84a5913bbbbe165
    SOURCE_DIR        ${SB_SOURCE_DIR}/lastools
    CONFIGURE_COMMAND ""
    CMAKE_COMMAND     ""
    CMAKE_GENERATOR   ""
    UPDATE_COMMAND    ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND     make -C LASlib -j$(nproc) CXXFLAGS='-std=c++11' && make -C src -j$(nproc) CXXFLAGS='-std=c++11' lasmerge
    INSTALL_COMMAND   mv ${SB_SOURCE_DIR}/lastools/bin/lasmerge ${SB_INSTALL_DIR}/bin
)
