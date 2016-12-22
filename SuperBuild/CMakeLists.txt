cmake_minimum_required(VERSION 3.1)

project(ODM-SuperBuild)

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
set(ODM_PCL_Version 1.7.2)
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
# Open Geometric Vision (OpenGV)
# Open Structure from Motion (OpenSfM)
# Clustering Views for Multi-view Stereo (CMVS)
# Catkin
# Ecto
#

set(custom_libs OpenGV
				OpenSfM
				CMVS
				Catkin
				Ecto
				PDAL
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

