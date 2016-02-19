project(odm_meshing)
cmake_minimum_required(VERSION 2.8)

# Set pcl dir to the input spedified with option -DPCL_DIR="path"
set(PCL_DIR "PCL_DIR-NOTFOUND" CACHE "PCL_DIR" "Path to the pcl installation directory")

# Add compiler options.
add_definitions(-Wall -Wextra)

# Find pcl at the location specified by PCL_DIR
find_package(PCL 1.7 HINTS "${PCL_DIR}/share/pcl-1.7")

# Add the PCL and Eigen include dirs. 
# Necessary since the PCL_INCLUDE_DIR variable set by find_package is broken.)
include_directories(${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR})
include_directories(${EIGEN_ROOT})

# Add source directory
aux_source_directory("./src" SRC_LIST)

# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})

# Link
target_link_libraries(odm_meshing ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SURFACE_LIBRARIES})

