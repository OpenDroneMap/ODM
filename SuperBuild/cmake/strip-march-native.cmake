# Removes -march=native from a CMakeLists.txt at patch time.
# Usage: cmake -DFILE=<path> -P strip-march-native.cmake
file(READ "${FILE}" _content)
string(REPLACE "-march=native" "" _content "${_content}")
file(WRITE "${FILE}" "${_content}")
