# Draco wraps executable link dependencies in GNU-linker -Wl,--start-group /
# --end-group for any Clang/GNU compiler. Apple's ld64 does not understand
# --start-group (and does not need it -- it rescans archives regardless of
# order), so exclude Apple from that branch.
# Usage: cmake -DFILE=<.../cmake/draco_targets.cmake> -P patch-draco-no-start-group.cmake
file(READ "${FILE}" _content)
string(REPLACE
  "if(CMAKE_CXX_COMPILER_ID MATCHES \"^Clang|^GNU\")"
  "if((CMAKE_CXX_COMPILER_ID MATCHES \"^Clang|^GNU\") AND NOT APPLE)"
  _content "${_content}")
file(WRITE "${FILE}" "${_content}")
