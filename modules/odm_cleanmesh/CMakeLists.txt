project(odm_cleanmesh)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

set (CMAKE_CXX_STANDARD 11)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# Add compiler options.
add_definitions(-Wall -Wextra)

# Add source directory
aux_source_directory("./src" SRC_LIST)
	
# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})
	
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES})
