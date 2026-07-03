cmake_minimum_required(VERSION 3.10)

project(ODM-SuperBuild)

# Setup SuperBuild root location
set(SB_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

################################
# Setup SuperBuild directories #
################################

# Setup location where source tar-balls are downloaded
set(SB_DOWNLOAD_DIR "${SB_ROOT_DIR}/download"
    CACHE PATH "Location where source tar-balls are (to be) downloaded.")
mark_as_advanced(SB_DOWNLOAD_DIR)

message(STATUS "SuperBuild files will be downloaded to: ${SB_DOWNLOAD_DIR}")


# Setup location where source tar-balls are located
set(SB_SOURCE_DIR "${SB_ROOT_DIR}/src"
    CACHE PATH "Location where source tar-balls are (will be).")
mark_as_advanced(SB_SOURCE_DIR)
set(SB_BUILD_DIR "${SB_ROOT_DIR}/build")

message(STATUS "SuperBuild source files will be extracted to: ${SB_SOURCE_DIR}")


# Setup location where source tar-balls are located
set(SB_INSTALL_DIR "${SB_ROOT_DIR}/install"
    CACHE PATH "Location where source tar-balls are (will be) installed.")
mark_as_advanced(SB_SOURCE_DIR)

message(STATUS "SuperBuild source files will be installed to: ${SB_INSTALL_DIR}")


# Setup location where binary files are located
set(SB_BINARY_DIR "${SB_ROOT_DIR}/build"
    CACHE PATH "Location where files are (will be) located.")
mark_as_advanced(SB_BINARY_DIR)

message(STATUS "SuperBuild binary files will be located to: ${SB_BINARY_DIR}")

if(NOT DEFINED ENV{CONDA_PREFIX})
    message(FATAL_ERROR
        "SuperBuild requires a pixi/conda environment (CONDA_PREFIX). "
        "Use: pixi install && pixi run build")
endif()

list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
set(CONDA_CMAKE_ARGS
    "-DCMAKE_PREFIX_PATH=$ENV{CONDA_PREFIX}"
    "-DPython3_ROOT_DIR=$ENV{CONDA_PREFIX}"
    "-DPython_ROOT_DIR=$ENV{CONDA_PREFIX}")
message(STATUS "Using conda prefix: $ENV{CONDA_PREFIX}")

# On Linux the conda-forge toolchain bakes an RPATH to the environment's lib
# into every binary. macOS clang does not, so libraries linked with an
# @rpath install name (e.g. conda boost) fail to load at runtime. Bake the
# same RPATH in explicitly so installed binaries locate the conda libraries.
if(APPLE)
    list(APPEND CONDA_CMAKE_ARGS
        "-DCMAKE_INSTALL_RPATH=$ENV{CONDA_PREFIX}/lib"
        "-DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON")
endif()

set(PYTHON_HOME "$ENV{CONDA_PREFIX}")
if(WIN32)
    set(PYTHON_EXE_PATH "${PYTHON_HOME}/python.exe")
else()
    set(PYTHON_EXE_PATH "${PYTHON_HOME}/bin/python")
endif()

# Propagate build type to ExternalProject sub-builds (MSVC defaults to Debug otherwise).
set(SB_EP_CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")

# Conda compiler-rt can inject clang -Wl flags; MSVC link.exe rejects them.
if(WIN32 AND DEFINED ENV{CONDA_PREFIX})
    list(APPEND SB_EP_CMAKE_ARGS
        "-DCMAKE_EXE_LINKER_FLAGS="
        "-DCMAKE_SHARED_LINKER_FLAGS="
        "-DCMAKE_MODULE_LINKER_FLAGS=")
endif()

# Path to additional CMake modules
set(CMAKE_MODULE_PATH ${SB_ROOT_DIR}/cmake)

include(ExternalProject)
include(ExternalProject-Setup)
include(Conda-Packages)

#########################################
# Conda-provided libraries (pixi.toml)  #
#########################################

odm_require_conda_package(GDAL)
odm_require_conda_package(OpenCV)
odm_require_conda_package(GFlags)
odm_require_conda_package(Ceres)
odm_require_conda_package(PDAL)

#########################################
# Source-built dependencies             #
#########################################

# ---------------------------------------------------------------------------------------------
# Open Structure from Motion (OpenSfM)
#

set(custom_libs OpenSfM
    Entwine
    MvsTexturing
    OpenMVS
    FPCFilter
    PyPopsift
    Obj2Tiles
    OpenPointClass
    RenderDEM
)

externalproject_add(mve
    GIT_REPOSITORY  https://github.com/OpenDroneMap/mve.git
    GIT_TAG         356
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${SB_SOURCE_DIR}/mve
    CMAKE_ARGS      ${SB_EP_CMAKE_ARGS} ${CONDA_CMAKE_ARGS}
    BUILD_IN_SOURCE 1
    INSTALL_COMMAND ""
)

foreach(lib ${custom_libs})
	SETUP_EXTERNAL_PROJECT_CUSTOM(${lib})
endforeach()

include(ProcessorCount)
if(DEFINED ENV{CMAKE_BUILD_PARALLEL_LEVEL})
  set(nproc $ENV{CMAKE_BUILD_PARALLEL_LEVEL})
else()
  ProcessorCount(nproc)
endif()

if (WIN32)
    find_program(MSBUILD_EXECUTABLE msbuild REQUIRED)
    if(NOT CMAKE_VS_PLATFORM_TOOLSET)
        set(CMAKE_VS_PLATFORM_TOOLSET "v143")
    endif()
    if(NOT CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION)
        if(DEFINED ENV{WindowsSDKVersion})
            string(STRIP "$ENV{WindowsSDKVersion}" _poisson_sdk_ver)
            string(REGEX REPLACE "\\\\+$" "" _poisson_sdk_ver "${_poisson_sdk_ver}")
            set(CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION "${_poisson_sdk_ver}")
        else()
            file(GLOB _poisson_sdk_dirs "C:/Program Files (x86)/Windows Kits/10/Include/10.0.*")
            list(SORT _poisson_sdk_dirs)
            list(REVERSE _poisson_sdk_dirs)
            if(_poisson_sdk_dirs)
                get_filename_component(_poisson_sdk_ver "${_poisson_sdk_dirs}" NAME)
                set(CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION "${_poisson_sdk_ver}")
            endif()
        endif()
    endif()
    if(NOT CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION)
        message(FATAL_ERROR "Windows SDK version not found. Run builds from pixi shell after MSVC activation, or set CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION.")
    endif()
    set(POISSON_BUILD_CMD
        "${MSBUILD_EXECUTABLE}"
        "${SB_SOURCE_DIR}/PoissonRecon/PoissonRecon.vcxproj"
        "/p:configuration=${CMAKE_BUILD_TYPE}"
        "/p:Platform=x64"
        "/p:PlatformToolset=${CMAKE_VS_PLATFORM_TOOLSET}"
        "/p:WindowsTargetPlatformVersion=${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION}")
    set (POISSON_BIN_PATH "x64/${CMAKE_BUILD_TYPE}/PoissonRecon.exe")
else()
    # Inherit pixi/conda compiler flags (LDFLAGS for -ljpeg, etc.) from the shell environment.
    set (POISSON_BUILD_CMD bash -lc "make -j${nproc} poissonrecon")
    set (POISSON_BIN_PATH "Linux/PoissonRecon")
    if (APPLE)
        set(POISSON_BUILD_CMD bash -lc "make -j${nproc} -f Makefile.macos poissonrecon COMPILER=${CMAKE_CXX_COMPILER} CONDA_PREFIX=$ENV{CONDA_PREFIX}")
    endif()
endif()
externalproject_add(poissonrecon
    GIT_REPOSITORY    https://github.com/OpenDroneMap/PoissonRecon.git
    GIT_TAG           290
    PREFIX            ${SB_BINARY_DIR}/PoissonRecon
    SOURCE_DIR        ${SB_SOURCE_DIR}/PoissonRecon
    UPDATE_COMMAND    ""
    PATCH_COMMAND     ${CMAKE_COMMAND} -P ${SB_ROOT_DIR}/cmake/apply-patch.cmake
                          ${SB_ROOT_DIR}/cmake/poissonrecon-sparsematrix-rows.patch
                          ${SB_ROOT_DIR}/cmake/poissonrecon-zlib-byte.patch
                          ${SB_ROOT_DIR}/cmake/poissonrecon-png-mac.patch
                          ${SB_ROOT_DIR}/cmake/poissonrecon-macos-makefile.patch
    CONFIGURE_COMMAND ""
    BUILD_IN_SOURCE 1
    BUILD_COMMAND    ${POISSON_BUILD_CMD}
    INSTALL_COMMAND  ${CMAKE_COMMAND} -E copy ${SB_SOURCE_DIR}/PoissonRecon/Bin/${POISSON_BIN_PATH} ${SB_INSTALL_DIR}/bin
)

externalproject_add(dem2mesh
    DEPENDS           gdal
    GIT_REPOSITORY    https://github.com/OpenDroneMap/dem2mesh.git
    GIT_TAG           334
    PREFIX            ${SB_BINARY_DIR}/dem2mesh
    SOURCE_DIR        ${SB_SOURCE_DIR}/dem2mesh
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${SB_EP_CMAKE_ARGS}
                      ${CONDA_CMAKE_ARGS}
)

externalproject_add(dem2points
    DEPENDS           gdal
    GIT_REPOSITORY    https://github.com/OpenDroneMap/dem2points.git
    GIT_TAG           master
    PREFIX            ${SB_BINARY_DIR}/dem2points
    SOURCE_DIR        ${SB_SOURCE_DIR}/dem2points
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${SB_EP_CMAKE_ARGS}
                      ${CONDA_CMAKE_ARGS}
)

externalproject_add(odm_orthophoto
    DEPENDS           opencv gdal
    GIT_REPOSITORY    https://github.com/OpenDroneMap/odm_orthophoto.git
    GIT_TAG           355
    PREFIX            ${SB_BINARY_DIR}/odm_orthophoto
    SOURCE_DIR        ${SB_SOURCE_DIR}/odm_orthophoto
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${SB_EP_CMAKE_ARGS}
                      ${CONDA_CMAKE_ARGS}
)

externalproject_add(fastrasterfilter
    DEPENDS           gdal
    GIT_REPOSITORY    https://github.com/OpenDroneMap/FastRasterFilter.git
    GIT_TAG           main
    PREFIX            ${SB_BINARY_DIR}/fastrasterfilter
    SOURCE_DIR        ${SB_SOURCE_DIR}/fastrasterfilter
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${SB_EP_CMAKE_ARGS}
                      ${CONDA_CMAKE_ARGS}
)

externalproject_add(lastools
    GIT_REPOSITORY    https://github.com/OpenDroneMap/LAStools.git
    GIT_TAG           250
    PREFIX            ${SB_BINARY_DIR}/lastools
    SOURCE_DIR        ${SB_SOURCE_DIR}/lastools
    CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${SB_EP_CMAKE_ARGS}
)

externalproject_add(draco
    GIT_REPOSITORY    https://github.com/google/draco
    GIT_SHALLOW       OFF
    GIT_TAG           e801aebf9665f6dcf3b6b0a604055a300ff85aa4
    PREFIX            ${SB_BINARY_DIR}/draco
    SOURCE_DIR        ${SB_SOURCE_DIR}/draco
    PATCH_COMMAND     ${CMAKE_COMMAND} -P ${SB_ROOT_DIR}/cmake/apply-patch.cmake ${SB_ROOT_DIR}/cmake/draco-no-start-group.patch
    CMAKE_ARGS        -DDRACO_TRANSCODER_SUPPORTED=ON
                      -DCMAKE_INSTALL_PREFIX:PATH=${SB_INSTALL_DIR}
                      ${SB_EP_CMAKE_ARGS}
                      ${CONDA_CMAKE_ARGS}
)
