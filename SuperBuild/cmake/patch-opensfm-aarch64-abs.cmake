# OpenSfM's triangulation.h uses std::abs<T>(det) on the __aarch64__ path.
# On Apple Silicon with libc++/clang this resolves to the std::complex overload
# of abs(), which calls std::hypot() on a ceres::Jet and fails to compile. The
# non-aarch64 path already uses abs(det), which resolves correctly via ADL
# (ceres::abs for Jets, std::abs for scalars). Exclude Apple from the aarch64
# branch so macOS arm64 uses the working code path; linux-aarch64 is unaffected.
# Usage: cmake -DFILE=<path/to/triangulation.h> -P patch-opensfm-aarch64-abs.cmake
file(READ "${FILE}" _content)
string(REPLACE
  "#ifdef __aarch64__"
  "#if defined(__aarch64__) && !defined(__APPLE__)"
  _content "${_content}")
file(WRITE "${FILE}" "${_content}")
