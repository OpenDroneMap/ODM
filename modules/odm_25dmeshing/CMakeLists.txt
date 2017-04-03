project(odm_25dmeshing)
cmake_minimum_required(VERSION 2.8)

# Link
find_package(CGAL QUIET COMPONENTS Core )

if ( CGAL_FOUND )

  find_package( Eigen3 QUIET )

  if( EIGEN3_FOUND )
    include_directories(${EIGEN3_INCLUDE_DIR})
    add_definitions(-DCGAL_EIGEN3_ENABLED)
  else()
    message(FATAL_ERROR "This program requires the Eigen3 library, and will not be compiled.")
  endif()

  include( ${CGAL_USE_FILE} )

  # Add compiler options.
  add_definitions(-Wall -Wextra)
	
  # Add source directory
  aux_source_directory("./src" SRC_LIST)
	
  # Add exectuteable
  add_executable(${PROJECT_NAME} ${SRC_LIST})
	
  target_link_libraries(odm_25dmeshing ${CGAL_LIBRARIES})
else()
  
    message(FATAL_ERROR "This program requires the CGAL library, and will not be compiled.")
  
endif()

