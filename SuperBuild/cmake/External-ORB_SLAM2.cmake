set(_proj_name orb_slam2)
set(_SB_BINARY_DIR "${SB_BINARY_DIR}/${_proj_name}")

ExternalProject_Add(${_proj_name}
  DEPENDS           opencv pangolin
  PREFIX            ${_SB_BINARY_DIR}
  TMP_DIR           ${_SB_BINARY_DIR}/tmp
  STAMP_DIR         ${_SB_BINARY_DIR}/stamp
  #--Download step--------------
  DOWNLOAD_DIR      ${SB_DOWNLOAD_DIR}
  URL               https://github.com/paulinus/ORB_SLAM2/archive/7c11f186a53a75560cd17352d327b0bc127a82de.zip
  #--Update/Patch step----------
  UPDATE_COMMAND    ""
  #--Configure step-------------
  SOURCE_DIR        ${SB_SOURCE_DIR}/${_proj_name}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
  #--Build step-----------------
  BINARY_DIR        ${_SB_BINARY_DIR}
  #--Install step---------------
  INSTALL_COMMAND    ""
  #--Output logging-------------
  LOG_DOWNLOAD      OFF
  LOG_CONFIGURE     OFF
  LOG_BUILD         OFF
)

# DBoW2
set(DBoW2_BINARY_DIR "${SB_BINARY_DIR}/DBoW2")
file(MAKE_DIRECTORY "${DBoW2_BINARY_DIR}")

ExternalProject_Add_Step(${_proj_name} build_DBoW2
  COMMAND make -j2
  DEPENDEES configure_DBoW2
  DEPENDERS configure
  WORKING_DIRECTORY ${DBoW2_BINARY_DIR}
  ALWAYS 1
)

ExternalProject_Add_Step(${_proj_name} configure_DBoW2
  COMMAND ${CMAKE_COMMAND} <SOURCE_DIR>/Thirdparty/DBoW2
    -DOpenCV_DIR=${SB_INSTALL_DIR}/share/OpenCV
    -DCMAKE_BUILD_TYPE=Release
  DEPENDEES download
  DEPENDERS build_DBoW2
  WORKING_DIRECTORY ${DBoW2_BINARY_DIR}
  ALWAYS 1
)

# g2o
set(g2o_BINARY_DIR "${SB_BINARY_DIR}/g2o")
file(MAKE_DIRECTORY "${g2o_BINARY_DIR}")

ExternalProject_Add_Step(${_proj_name} build_g2o
  COMMAND make -j2
  DEPENDEES configure_g2o
  DEPENDERS configure
  WORKING_DIRECTORY ${g2o_BINARY_DIR}
  ALWAYS 1
)

ExternalProject_Add_Step(${_proj_name} configure_g2o
  COMMAND ${CMAKE_COMMAND} <SOURCE_DIR>/Thirdparty/g2o
    -DCMAKE_BUILD_TYPE=Release
  DEPENDEES download
  DEPENDERS build_g2o
  WORKING_DIRECTORY ${g2o_BINARY_DIR}
  ALWAYS 1
)

# Uncompress Vocabulary
ExternalProject_Add_Step(${_proj_name} uncompress_vocabulary
  COMMAND tar -xf ORBvoc.txt.tar.gz
  DEPENDEES download
  DEPENDERS configure
  WORKING_DIRECTORY <SOURCE_DIR>/Vocabulary
  ALWAYS 1
)
