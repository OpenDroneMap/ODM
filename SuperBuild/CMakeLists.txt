cmake_minimum_required(VERSION 2.8)

project(ODM-SuperBuild)

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake)

include(ExternalProject)

# Setup location where source tar-balls are downloaded
set (DOWNLOAD_LOCATION "${CMAKE_SOURCE_DIR}/Downloads"
     CACHE PATH "Location where source tar-balls are (to be) downloaded.")
mark_as_advanced(DOWNLOAD_LOCATION)

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
set(ODM_OpenCV_RequiredVersion 2.4.11)

find_package( OpenCV ${ODM_OpenCV_RequiredVersion} EXACT QUIET)

if(${OpenCV_FOUND})
  message(STATUS "OpenCV found --- ${OpenCV_VERSION} ( < ${ODM_OpenCV_RequiredVersion} )")
  set(OPENCV_DIR ${OpenCV_DIR})
else()
  message(STATUS "OpenCV not found --> Adding External project - OpenCV ${ODM_OpenCV_RequiredVersion}")
  include(External-OpenCV)
endif()


# ---------------------------------------------------------------------------------------------
# Point Cloud Library (PCL)
#
set(ODM_PCL_RequiredVersion 1.7.2)

find_package( PCL ${ODM_PCL_RequiredVersion} EXACT QUIET)

if(${PCL_FOUND})
  message(STATUS "PCL found --- ${PCL_VERSION} ( < ${ODM_PCL_RequiredVersion} )")
  set(PCL_DIR ${PCL_DIR})
else()
  message(STATUS "PCL not found --> Adding External project - PCL ${ODM_PCL_RequiredVersion}")
  include(External-PCL)
endif()
