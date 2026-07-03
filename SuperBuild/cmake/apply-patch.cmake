# Applies one or more patches with `git apply`, skipping if they're already
# applied.
#
# CMake's ExternalProject_Add reruns PATCH_COMMAND whenever its literal
# command line changes (e.g. after pulling in a SuperBuild change), not just
# on a fresh checkout. `git apply` errors out if the patch is already
# reflected in the working tree, so a plain `git apply` in PATCH_COMMAND
# fails on that rerun. Skip the apply when reversing the patch would be a
# clean no-op, since that means it's already applied.
#
# Usage: cmake -P apply-patch.cmake <patch1> [patch2 ...]
# (run with the external project's source dir as the working directory)
#
# Patches are passed as trailing positional arguments (CMAKE_ARGV3+) rather
# than a single -D list, since ExternalProject_Add re-splits `;`-joined -D
# values on their way to the generated build command.

set(_patches)
math(EXPR _last_arg "${CMAKE_ARGC} - 1")
foreach(_i RANGE 3 ${_last_arg})
  list(APPEND _patches "${CMAKE_ARGV${_i}}")
endforeach()

execute_process(
  COMMAND git apply --whitespace=nowarn --reverse --check ${_patches}
  RESULT_VARIABLE _already_applied
  OUTPUT_QUIET
  ERROR_QUIET
)
if(_already_applied EQUAL 0)
  return()
endif()

execute_process(
  COMMAND git apply --whitespace=nowarn ${_patches}
  RESULT_VARIABLE _apply_result
)
if(NOT _apply_result EQUAL 0)
  message(FATAL_ERROR "git apply failed for: ${_patches}")
endif()
