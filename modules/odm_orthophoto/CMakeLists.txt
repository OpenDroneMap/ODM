project(odm_orthophoto)
cmake_minimum_required(VERSION 2.8)

# Set pcl dir to the input spedified with option -DPCL_DIR="path"
set(PCL_DIR "PCL_DIR-NOTFOUND" CACHE "PCL_DIR" "Path to the pcl installation directory")
set(OPENCV_DIR "OPENCV_DIR-NOTFOUND" CACHE "OPENCV_DIR" "Path to the OPENCV installation directory")

# Add compiler options.
add_definitions(-Wall -Wextra)

# Find pcl at the location specified by PCL_DIR
find_package(VTK 6.0 REQUIRED)
find_package(PCL 1.8 HINTS "${PCL_DIR}/share/pcl-1.8" REQUIRED)

# Find OpenCV at the default location
find_package(OpenCV HINTS "${OPENCV_DIR}" REQUIRED)

# Only link with required opencv modules.
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui)

# Add the PCL, Eigen and OpenCV include dirs.
# Necessary since the PCL_INCLUDE_DIR variable set by find_package is broken.)
include_directories(${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR})
include_directories(${EIGEN_ROOT})
include_directories(${OpenCV_INCLUDE_DIRS})

#library_directories(${OpenCV_LIBRARY_DIRS})

# Add source directory
aux_source_directory("./src" SRC_LIST)

# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(odm_orthophoto ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${OpenCV_LIBS})