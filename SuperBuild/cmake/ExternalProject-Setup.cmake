set(FORCE_BUILD_LIB_MSG "--- Adding internal version")

macro(SETUP_EXTERNAL_PROJECT_CUSTOM name)
  message(STATUS "${name} ${FORCE_BUILD_LIB_MSG}")
  include(External-${name})
endmacro()