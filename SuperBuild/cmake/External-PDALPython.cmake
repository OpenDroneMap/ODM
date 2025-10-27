set(_proj_name pdal-python)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

if (WIN32)
  set(PP_EXTRA_ARGS -DPython3_EXECUTABLE=${PYTHON_EXE_PATH}
                    -DPython3_ROOT_DIR=${PYTHON_HOME}
                    -DPython3_NumPy_INCLUDE_DIRS=${PYTHON_HOME}/Lib/site-packages/numpy/_core/include)
else()
  set(PP_EXTRA_ARGS -DPython3_EXECUTABLE=${PYTHON_EXE_PATH}
                    -DPython3_NumPy_INCLUDE_DIRS=${PYTHON_HOME}/lib/python3.12/site-packages/numpy/_core/include)
endif()

ExternalProject_Add(${_proj_name}
  DEPENDS           pdal
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  GIT_REPOSITORY    https://github.com/PDAL/python
  GIT_TAG           6791a880a87e95f7318e99acfb4a10186379c5dd
  #--Update/Patch step----------
  UPDATE_COMMAND    git submodule update --init --recursive
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DPDAL_DIR=${SB_INSTALL_DIR}/lib/cmake/PDAL
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}/lib/python3.12/dist-packages
    ${WIN32_CMAKE_ARGS}
    ${PP_EXTRA_ARGS}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_DIR       ${SB_INSTALL_DIR}
  INSTALL_COMMAND   ${CMAKE_COMMAND} --build <BINARY_DIR> --target install
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SB_SOURCE_DIR}/${_proj_name}/src/pdal/__init__.py ${SB_INSTALL_DIR}/lib/python3.12/dist-packages/pdal
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SB_SOURCE_DIR}/${_proj_name}/src/pdal/pipeline.py ${SB_INSTALL_DIR}/lib/python3.12/dist-packages/pdal
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SB_SOURCE_DIR}/${_proj_name}/src/pdal/drivers.py ${SB_INSTALL_DIR}/lib/python3.12/dist-packages/pdal
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)
