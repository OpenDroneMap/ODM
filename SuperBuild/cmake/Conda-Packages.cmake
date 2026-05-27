# Conda/pixi packages required at configure time (see pixi.toml).

function(odm_require_conda_package name)
  find_package(${name} REQUIRED)
  if(${name}_FOUND AND ${name}_VERSION)
    message(STATUS "${name} ${${name}_VERSION} found (conda)")
  else()
    message(STATUS "${name} found (conda)")
  endif()
  string(TOLOWER "${name}" _odm_target)
  if(NOT TARGET ${_odm_target})
    add_custom_target(${_odm_target})
  endif()
endfunction()
