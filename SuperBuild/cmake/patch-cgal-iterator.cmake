# CGAL 5.6.1 defines a "safe bool" conversion operator on three half-edge
# iterators (Halfedge_around_{source,target,face}_iterator) that calls
# this->base() -- a member those iterators do not have. Clang 19 (conda-forge
# toolchain on macOS) diagnoses this as an error. Upstream removed these
# operators in CGAL #8313 (merged for 5.6.2 / 6.0); conda-forge has no 5.6.2,
# so patch the header in place. The three blocks are byte-identical and unique
# (other operator bool blocks use base_reference()/g), so one removal is safe.
# Usage: cmake -DFILE=<.../CGAL/boost/graph/iterator.h> -P patch-cgal-iterator.cmake
if(EXISTS "${FILE}")
  file(READ "${FILE}" _content)
  string(REPLACE
"  explicit operator bool() const
  {
    return (! (this->base() == nullptr));
  }
"
"" _content "${_content}")
  file(WRITE "${FILE}" "${_content}")
endif()
