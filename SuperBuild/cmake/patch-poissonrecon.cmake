# macOS / clang 19 fixes for PoissonRecon. Pass the source dir as -DSRC=<dir>.
# Usage: cmake -DSRC=<PoissonRecon source dir> -P patch-poissonrecon.cmake

# 1. SparseMatrix.inl uses `A.rows` (a member function) without calling it in two
#    loop conditions. Clang rejects this; add the missing parentheses.
set(_f "${SRC}/Src/SparseMatrix.inl")
file(READ "${_f}" _c)
string(REPLACE
  "for( size_t i=0 ; i<A.rows ; i++ )"
  "for( size_t i=0 ; i<A.rows() ; i++ )"
  _c "${_c}")
file(WRITE "${_f}" "${_c}")

# 2. The bundled zlib's zconf.h skips defining `Byte` on macOS, expecting Apple's
#    MacTypes.h to supply it -- but it is never included here, so `Byte` is
#    undefined. Define it unconditionally (a same-type typedef redefinition is
#    harmless where MacTypes.h is present).
set(_f "${SRC}/ZLIB/zconf.h")
file(READ "${_f}" _c)
string(REPLACE
"#if !defined(MACOS) && !defined(TARGET_OS_MAC)
typedef unsigned char  Byte;  /* 8 bits */
#endif"
"typedef unsigned char  Byte;  /* 8 bits */"
  _c "${_c}")
file(WRITE "${_f}" "${_c}")

# 3. PoissonRecon's headers use the `template` keyword without a following
#    template-argument list, which clang 19 makes a hard error. Suppress it for
#    the macOS build (the same pattern compiles fine on gcc).
set(_f "${SRC}/Makefile.macos")
file(READ "${_f}" _c)
string(REPLACE
  "-Wno-invalid-offsetof"
  "-Wno-invalid-offsetof -Wno-missing-template-arg-list-after-template-kw"
  _c "${_c}")
# Makefile.macos was written for Homebrew gcc/libgomp. Pixi/conda macOS builds
# use clang + llvm-openmp (libomp), so point at CONDA_PREFIX instead.
string(REPLACE
  "CFLAGS += -fopenmp"
  "CFLAGS += -fopenmp=libomp"
  _c "${_c}")
string(REPLACE
  "LFLAGS += -lgomp -lstdc++ -lpthread -L/opt/homebrew/lib"
  "LFLAGS += -lomp -lpthread -L$(CONDA_PREFIX)/lib"
  _c "${_c}")
string(REPLACE
  "INCLUDE = . -I/opt/homebrew/include"
  "INCLUDE = . -I$(CONDA_PREFIX)/include"
  _c "${_c}")
string(REPLACE
  "COMPILER ?= gcc"
  "COMPILER ?= gcc\nCONDA_PREFIX ?="
  _c "${_c}")
file(WRITE "${_f}" "${_c}")

# 4. The bundled libpng (libpng 1.2.x era) treats TARGET_OS_MAC as "Classic Mac
#    OS" and then includes the long-gone <fp.h>. Modern macOS SDKs define
#    TARGET_OS_MAC=1 on every Apple platform, so this misfires and breaks the
#    build. Drop it from the MACOS detection so macOS takes the same standard
#    <sys/types.h>/<math.h> paths as Linux/Windows. (Same root cause as the zlib
#    TARGET_OS_MAC fix above; on non-Apple platforms this is a no-op.)
set(_f "${SRC}/PNG/pngconf.h")
file(READ "${_f}" _c)
string(REPLACE
  "      defined(THINK_C) || defined(__SC__) || defined(TARGET_OS_MAC)"
  "      defined(THINK_C) || defined(__SC__)"
  _c "${_c}")
file(WRITE "${_f}" "${_c}")
