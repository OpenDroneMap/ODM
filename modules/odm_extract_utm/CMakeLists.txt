project(odm_extract_utm)
cmake_minimum_required(VERSION 2.8)

set(PROJ4_INCLUDE_DIR "/usr/include/" CACHE "PROJ4_INCLUDE_DIR" "Path to the proj4 inlcude directory")

find_library(PROJ4_LIBRARY "libproj.so" PATHS "/usr/lib" "/usr/lib/x86_64-linux-gnu")

# Add compiler options.
add_definitions(-Wall -Wextra)

# Add source directory
aux_source_directory("./src" SRC_LIST)

# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})

# Link
target_link_libraries(${PROJECT_NAME} ${PROJ4_LIBRARY})

