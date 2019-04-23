if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Add ODM sub-modules
add_subdirectory(odm_georef)
add_subdirectory(odm_orthophoto)
add_subdirectory(odm_cleanmesh)
add_subdirectory(odm_filterpoints)

if (ODM_BUILD_SLAM)
  add_subdirectory(odm_slam)
endif ()
