cmake_minimum_required(VERSION 2.8)

project(ODM-SuperBuild)

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake)

include(ExternalProject)
include(ExternalProject-Setup)

# Setup location where source tar-balls are downloaded
set (SB_DOWNLOAD_LOCATION "${CMAKE_SOURCE_DIR}/Downloads"
     CACHE PATH "Location where source tar-balls are (to be) downloaded.")
mark_as_advanced(DOWNLOAD_LOCATION)

# Setup where the files will be installed
set(SB_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Check if CMAKE_INSTALL_PREFIX is set by default
if(CMAKE_INSTALL_PREFIX STREQUAL "/usr/local")
  message(WARNING
    "The CMAKE_INSTALL_PREFIX variable seems to be set by default : "
    "${CMAKE_INSTALL_PREFIX}. Be aware that this directory will be used"
    " during the build (even without calling the install target). Please "
    "make sure you want to use this directory as the SuperBuild output.")
endif()


#############################
# Download third party libs #
#############################

# ---------------------------------------------------------------------------------------------
# OpenCV
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