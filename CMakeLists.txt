cmake_minimum_required(VERSION 2.8)

project(OpenDroneMap)

#TODO(edgar): add option in order to point to CMAKE_PREFIX_PATH
# e.g. ODM_SB_INSTALL_PATH

# Add ODM sub-modules
add_subdirectory(modules/odm_extract_utm)
add_subdirectory(modules/odm_georef)
add_subdirectory(modules/odm_meshing)
add_subdirectory(modules/odm_orthophoto)
add_subdirectory(modules/odm_texturing)

