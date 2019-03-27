project(odm_filterpoints)
cmake_minimum_required(VERSION 2.8)

# Add compiler options.
add_definitions(-Wall -Wextra -Wconversion -pedantic -std=c++11)

# PDAL and jsoncpp
find_package(PDAL REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/../../SuperBuild/src/pdal/vendor/jsoncpp/dist")
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

# Add source directory
aux_source_directory("./src" SRC_LIST)

# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})

# Link
target_link_libraries(${PROJECT_NAME} jsoncpp ${PDAL_LIBRARIES})
