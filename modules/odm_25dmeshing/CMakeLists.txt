project(odm_25dmeshing)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

set (CMAKE_CXX_STANDARD 11)
#set(VTK_DIR "/data/packages/VTK-7.1.1-build")
set(VTK_SMP_IMPLEMENTATION_TYPE TBB)
find_package(VTK 7.1.1 REQUIRED)
include(${VTK_USE_FILE})

# Add compiler options.
#add_definitions(-Wall -Wextra -O0 -g3)
add_definitions(-Wall -Wextra)

# Add source directory
aux_source_directory("./src" SRC_LIST)
	
# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})
	
target_link_libraries(odm_25dmeshing ${VTK_LIBRARIES})

