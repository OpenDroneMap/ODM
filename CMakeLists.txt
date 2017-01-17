cmake_minimum_required(VERSION 2.8)

project(OpenDroneMap C CXX)

# TODO(edgar): add option in order to point to CMAKE_PREFIX_PATH
# if we want to build SuperBuild in an external directory.
# It is assumed that SuperBuild have been compiled.

# Set third party libs location
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/SuperBuild/install")

# move binaries to the same bin directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(ODM_BUILD_SLAM "Build SLAM module" OFF)

# Add ODM sub-modules
add_subdirectory(modules)
