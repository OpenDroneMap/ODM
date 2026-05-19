# When CONDA_PREFIX is set (pixi), prefer conda-forge binaries over SuperBuild source builds.

set(ODM_PDAL_DIR "${SB_INSTALL_DIR}/lib/cmake/PDAL")
set(ODM_USE_CONDA_PDAL FALSE)
set(ODM_USE_CONDA_UNTWINE FALSE)

if(DEFINED ENV{CONDA_PREFIX})
  set(_conda_prefix "$ENV{CONDA_PREFIX}")

  find_package(PDAL CONFIG QUIET
    PATHS "${_conda_prefix}/lib/cmake/PDAL"
          "${_conda_prefix}/Library/lib/cmake/PDAL"
    NO_DEFAULT_PATH)

  if(PDAL_FOUND OR TARGET PDAL::pdal)
    if(PDAL_FOUND AND PDAL_VERSION)
      message(STATUS "PDAL ${PDAL_VERSION} found (conda)")
    else()
      message(STATUS "PDAL found (conda)")
    endif()
    set(ODM_USE_CONDA_PDAL TRUE)
    if(PDAL_DIR)
      set(ODM_PDAL_DIR "${PDAL_DIR}")
    elseif(EXISTS "${_conda_prefix}/lib/cmake/PDAL/PDALConfig.cmake")
      set(ODM_PDAL_DIR "${_conda_prefix}/lib/cmake/PDAL")
    elseif(EXISTS "${_conda_prefix}/Library/lib/cmake/PDAL/PDALConfig.cmake")
      set(ODM_PDAL_DIR "${_conda_prefix}/Library/lib/cmake/PDAL")
    endif()
    if(NOT TARGET pdal)
      add_custom_target(pdal)
    endif()
    if(NOT TARGET laszip)
      add_custom_target(laszip)
    endif()
    if(NOT TARGET hexer)
      add_custom_target(hexer)
    endif()
    if(NOT TARGET pdal-python)
      add_custom_target(pdal-python)
    endif()
  endif()

  find_program(ODM_UNTWINE_EXE
    NAMES untwine
    HINTS "${_conda_prefix}/bin" "${_conda_prefix}/Library/bin"
    NO_DEFAULT_PATH)
  if(ODM_UNTWINE_EXE)
    message(STATUS "untwine found (conda): ${ODM_UNTWINE_EXE}")
    set(ODM_USE_CONDA_UNTWINE TRUE)
    if(NOT TARGET untwine)
      add_custom_target(untwine)
    endif()
  endif()
endif()

if(NOT DEFINED ODM_PDAL_DIR)
  set(ODM_PDAL_DIR "${SB_INSTALL_DIR}/lib/cmake/PDAL")
endif()
