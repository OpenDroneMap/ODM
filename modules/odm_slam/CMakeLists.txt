project(odm_slam)
cmake_minimum_required(VERSION 2.8)

# Set opencv dir to the input spedified with option -DOPENCV_DIR="path"
set(OPENCV_DIR "OPENCV_DIR-NOTFOUND" CACHE "OPENCV_DIR" "Path to the opencv installation directory")

# Add compiler options.
add_definitions(-Wall -Wextra)

# Find pcl at the location specified by PCL_DIR
find_package(VTK 6.0 REQUIRED)
find_package(PCL 1.8 HINTS "${PCL_DIR}/share/pcl-1.8" REQUIRED)

# Find OpenCV at the default location
find_package(OpenCV HINTS "${OPENCV_DIR}" REQUIRED)

# Only link with required opencv modules.
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui)

# Add the Eigen and OpenCV include dirs.
# Necessary since the PCL_INCLUDE_DIR variable set by find_package is broken.)
include_directories(${EIGEN_ROOT})
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")

set(PANGOLIN_ROOT ${CMAKE_BINARY_DIR}/../SuperBuild/install)

set(ORB_SLAM_ROOT ${CMAKE_BINARY_DIR}/../SuperBuild/src/orb_slam2)

include_directories(${EIGEN_ROOT})
include_directories(${ORB_SLAM_ROOT})
include_directories(${ORB_SLAM_ROOT}/include)
link_directories(${PANGOLIN_ROOT}/lib)
link_directories(${ORB_SLAM_ROOT}/lib)

# Add source directory
aux_source_directory("./src" SRC_LIST)

# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(odm_slam ${OpenCV_LIBS} ORB_SLAM2 pangolin)
