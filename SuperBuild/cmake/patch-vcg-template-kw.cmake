# VCG's selection.h calls Allocator<>::template IsValidHandle(*_m, vsH) without
# explicit template arguments. Clang 19 requires the `template` disambiguator to
# be followed by a template-id, so it rejects this. IsValidHandle is templated on
# the attribute type (see quality.h, which calls IsValidHandle<ScalarType>), and
# vsH is a bool attribute handle, so supply the explicit <bool> argument.
# Usage: cmake -DFILE=<.../vcg/.../update/selection.h> -P patch-vcg-template-kw.cmake
file(READ "${FILE}" _content)
string(REPLACE
  "::template IsValidHandle(*_m, vsH)"
  "::template IsValidHandle<bool>(*_m, vsH)"
  _content "${_content}")
file(WRITE "${FILE}" "${_content}")
