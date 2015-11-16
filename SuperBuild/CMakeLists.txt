cmake_minimum_required(VERSION 2.8)

project(ODM-SuperBuild)

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(ExternalProject)
include(ExternalProject-Setup)


################################
# Setup SuperBuild directories #
################################

# Setup SuperBuild root location
set(SB_ROOT_DIR ${CMAKE_SOURCE_DIR})

# Setup location where source tar-balls are downloaded
set(SB_DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/download"
    CACHE PATH "Location where source tar-balls are (to be) downloaded.")
mark_as_advanced(SB_DOWNLOAD_DIR)

message(STATUS "SuperBuild files will be downloaded to: ${SB_DOWNLOAD_DIR}")


# Setup location where source tar-balls are located
set(SB_SOURCE_DIR "${CMAKE_SOURCE_DIR}/src"
    CACHE PATH "Location where source tar-balls are (will be).")
mark_as_advanced(SB_SOURCE_DIR)

message(STATUS "SuperBuild source files will be extracted to: ${SB_SOURCE_DIR}")


# Setup location where source tar-balls are located
set(SB_INSTALL_DIR "${CMAKE_SOURCE_DIR}/install"
    CACHE PATH "Location where source tar-balls are (will be) installed.")
mark_as_advanced(SB_SOURCE_DIR)

message(STATUS "SuperBuild source files will be installed to: ${SB_INSTALL_DIR}")


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
# Ceres Solver
#
set(ODM_Ceres_Version 1.10.0)
option(ODM_BUILD_Ceres "Force to build Ceres library" OFF)

SETUP_EXTERNAL_PROJECT(Ceres ${ODM_Ceres_Version} ${ODM_BUILD_Ceres})


# ---------------------------------------------------------------------------------------------
# Open Geometric Vision (OpenGV)
#

SETUP_EXTERNAL_PROJECT_CUSTOM(OpenVG)


# ---------------------------------------------------------------------------------------------
# Open Structure from Motion (OpenSfM)
#

SETUP_EXTERNAL_PROJECT_CUSTOM(OpenSfM)


# ---------------------------------------------------------------------------------------------
# Clustering Views for Multi-view Stereo (CMVS)
#

SETUP_EXTERNAL_PROJECT_CUSTOM(CMVS)