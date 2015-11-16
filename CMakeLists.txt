cmake_minimum_required(VERSION 2.8)

project(OpenDroneMap)

#TODO(edgar): add option in order to point to CMAKE_PREFIX_PATH
# e.g. ODM_SB_INSTALL_PATH

# Build SuperBuild
add_subdirectory(SuperBuild)

# Add ODM sub-modules
#add_subdirectory(modules)
