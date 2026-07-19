# Copy OpenSfM's compiled py* extension modules from the build dir into the
# package next to __init__.py so `import opensfm` can find them. scikit-build-core
# would normally do this at packaging time, but we build with raw cmake + make.
# Compatible with linux/win/macOS.
file(GLOB_RECURSE _osfm_mods
  "${OPENSFM_BINARY_DIR}/py*.so"
  "${OPENSFM_BINARY_DIR}/py*.pyd"
  "${OPENSFM_BINARY_DIR}/py*.dylib")

if(NOT _osfm_mods)
  message(FATAL_ERROR
    "No OpenSfM extension modules (py*.so/.pyd/.dylib) found under "
    "${OPENSFM_BINARY_DIR} - cannot make the opensfm package importable.")
endif()

file(COPY ${_osfm_mods} DESTINATION "${OPENSFM_PACKAGE_DIR}")

foreach(_m ${_osfm_mods})
  get_filename_component(_name "${_m}" NAME)
  message(STATUS "OpenSfM: installed ${_name} -> ${OPENSFM_PACKAGE_DIR}")
endforeach()
