set(ADD_INTERNAL_LIB_MSG "--- Adding internal version")
set(FORCE_BUILD_LIB_MSG "force build ${ADD_INTERNAL_LIB_MSG}")

macro(SETUP_EXTERNAL_PROJECT name version force_build)

  if(NOT ${force_build})

    find_package(${name} ${version} EXACT QUIET)

    if(${${name}_FOUND})
      message(STATUS "${name} ${${name}_VERSION} found")
      set(${name}_DIR ${${name}_DIR})
    else()
      message(STATUS "${name} ${version} not found ${ADD_INTERNAL_LIB_MSG}")
      include(External-${name})
    endif()
  else()
    message(STATUS "${name} ${version} ${FORCE_BUILD_LIB_MSG}")
    include(External-${name})
  endif()

endmacro()

macro(SETUP_EXTERNAL_PROJECT_CUSTOM name)
  message(STATUS "${name} ${FORCE_BUILD_LIB_MSG}")
  include(External-${name})
endmacro()