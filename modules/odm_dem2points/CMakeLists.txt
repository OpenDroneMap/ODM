project(odm_dem2points)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

set (CMAKE_CXX_STANDARD 11)
find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})

# Add compiler options.
add_definitions(-Wall -Wextra)

# Add source directory
aux_source_directory("./src" SRC_LIST)
	
# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})
	
target_link_libraries(${PROJECT_NAME} ${GDAL_LIBRARY})
