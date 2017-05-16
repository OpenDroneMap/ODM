project(odm_25dmeshing)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

# Set pcl dir to the input spedified with option -DCGAL_DIR="path"
set(CGAL_DIR "CGAL_DIR-NOTFOUND" CACHE "CGAL_DIR" "Path to the CGAL installation directory")

# Link
find_package(CGAL COMPONENTS Core HINTS "${CGAL_DIR}")

if ( CGAL_FOUND )

  find_package( Eigen3 )

  if( EIGEN3_FOUND )
    include_directories(${EIGEN3_INCLUDE_DIR})
    add_definitions(-DCGAL_EIGEN3_ENABLED)
  else()
    message(FATAL_ERROR "This program requires the Eigen3 library, and will not be compiled.")
  endif()

  include( ${CGAL_USE_FILE} )

  find_package( TBB )
  if( TBB_FOUND )
    include(${TBB_USE_FILE})
  else()
    message(WARNING "TBB not found, parallel processing will be disabled.")
  endif()

  # Add compiler options.
  add_definitions(-Wall -Wextra -std=c++11)
	
  # Add source directory
  aux_source_directory("./src" SRC_LIST)
	
  # Add exectuteable
  add_executable(${PROJECT_NAME} ${SRC_LIST})
	
  target_link_libraries(odm_25dmeshing ${CGAL_LIBRARIES} ${TBB_LIBRARIES})
else()
  
    message(FATAL_ERROR "This program requires the CGAL library, and will not be compiled.")
  
endif()

