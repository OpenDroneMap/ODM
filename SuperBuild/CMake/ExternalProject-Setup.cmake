function(SETUP_EXTERNAL_PROJECT name version force_build)

  set(ADD_LIB_MSG "--- Adding External project")

  if(NOT ${force_build})

    find_package(${name} ${version} EXACT QUIET)

    if(${${name}_FOUND})
      message(STATUS "${name} ${${name}_VERSION} found")
      set(${name}_DIR ${${name}_DIR})
    else()
      message(STATUS "${name} ${version} not found ${ADD_LIB_MSG}")
      include(External-${name})
    endif()
  else()
    message(STATUS "${name} ${version} force build ${ADD_LIB_MSG}")
    include(External-${name})
  endif()

endfunction()

macro(SETUP_EXTERNAL_PROJECT_CUSTOM name)
  set(ADD_LIB_MSG "--- Adding External project")
  message(STATUS "${name} force build ${ADD_LIB_MSG}")
  include(External-${name})
endmacro()