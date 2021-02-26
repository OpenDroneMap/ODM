if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Add ODM sub-modules
add_subdirectory(odm_orthophoto)
add_subdirectory(odm_cleanmesh)
