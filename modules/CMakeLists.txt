# Add ODM sub-modules
add_subdirectory(odm_extract_utm)
add_subdirectory(odm_georef)
add_subdirectory(odm_meshing)
add_subdirectory(odm_orthophoto)
add_subdirectory(odm_texturing)
if (ODM_BUILD_SLAM)
  add_subdirectory(odm_slam)
endif ()
