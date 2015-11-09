cmake_minimum_required(VERSION 2.8.9)

project(OpenDroneMap)

set(ODM_SOURCE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(ODM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(ODM_BINARY_DIR  ${PROJECT_BINARY_DIR}/bin)
set(ODM_LIBRARY_DIR ${PROJECT_BINARY_DIR}/lib)

set(ODM_THIRD_PARTY_DIR ${ODM_SOURCE_DIR}/third_party)

set(URL http://ftp.gnu.org/gnu/parallel/parallel-20141022.tar.bz2)
set(FILE_NAME parallel.tar.bz2)
set(FILE_DIR ${ODM_THIRD_PARTY_DIR}/parallel)
set(FILE_PATH ${FILE_DIR}/${FILE_NAME})

message(STATUS "downloading ... ${FILE_NAME}")

file(DOWNLOAD ${URL} ${FILE_PATH}
     EXPECTED_MD5 c01f53f9f6cc721a81591308f9e689c4
     STATUS status
     LOG log)

message(STATUS "downloading ... done")

message(STATUS "extracting ... ${FILE_PATH}")

if(NOT EXISTS "${FILE_PATH}")
  message(FATAL_ERROR "error: file to extract does not exist: '${FILE_NAME}'")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} -E tar xjf ${FILE_PATH}
                WORKING_DIRECTORY ${FILE_DIR}
                RESULT_VARIABLE rv)