# mvs-texturing bundles its own Eigen 3.3.2, which fails to compile with clang 19
# (Transpose<TranspositionsBase>::derived() does not exist). Bump the vendored
# copy to Eigen 3.4.0, which fixes this. Patches elibs/CMakeLists.txt in place.
# Usage: cmake -DFILE=elibs/CMakeLists.txt -P patch-mvstex-eigen.cmake
file(READ "${FILE}" _content)
string(REPLACE
  "archive/3.3.2/eigen-3.3.2.tar.gz"
  "archive/3.4.0/eigen-3.4.0.tar.gz"
  _content "${_content}")
string(REPLACE
  "02edfeec591ae09848223d622700a10b"
  "4c527a9171d71a72a9d4186e65bea559"
  _content "${_content}")
file(WRITE "${FILE}" "${_content}")
