project(odm_georef)
cmake_minimum_required(VERSION 2.8)

# Set pcl dir to the input spedified with option -DPCL_DIR="path"
set(PCL_DIR "PCL_DIR-NOTFOUND" CACHE "PCL_DIR" "Path to the pcl installation directory")
set(OPENCV_DIR "OPENCV_DIR-NOTFOUND" CACHE "OPENCV_DIR" "Path to the opencv installation directory")
set(PROJ4_INCLUDE_DIR "/usr/include/" CACHE "PROJ4_INCLUDE_DIR" "Path to the proj4 inlcude directory")
set(PROJ4_LIBRARY "/usr/lib/libproj.so" CACHE "PROJ4_LIBRARY" "Path to the proj4 library directory")

# Add compiler options.
add_definitions(-Wall -Wextra -Wconversion -pedantic)
#add_definitions(-pedantic -pedantic-errors -Wall -Wextra -Werror -Wfatal-errors -Wabi -Wctor-dtor-privacy -Wnon-virtual-dtor -Wreorder -Weffc++ -Wstrict-null-sentinel -Wnon-template-friend -Wold-style-cast -Woverloaded-virtual -Wpmf-conversions -Wsign-promo -Waddress -Warray-bounds -Wattributes -Wbuiltin-macro-redefined -Wc++0x-compat -Wcast-align -Wcast-qual -Wchar-subscripts -Wclobbered -Wcomment -Wconversion -Wcoverage-mismatch -Wdeprecated -Wdeprecated-declarations -Wdisabled-optimization -Wdiv-by-zero -Wempty-body -Wenum-compare -Wendif-labels -Wfatal-errors -Wfloat-equal -Wformat -Wformat=2 -Wformat-contains-nul -Wformat-extra-args -Wformat-nonliteral -Wformat-security -Wformat-y2k -Wignored-qualifiers -Winit-self -Wint-to-pointer-cast -Winvalid-offsetof -Winvalid-pch -Wlogical-op -Wmain -Wvariadic-macros -Wmissing-braces -Wmissing-field-initializers -Wmissing-include-dirs -Wmissing-noreturn -Wvla -Wmultichar -Wfatal-errors -Wnonnull -Woverflow -Woverlength-strings -Wpacked -Wpacked-bitfield-compat -Wparentheses -Wpointer-arith -Wredundant-decls -Wsequence-point -Wshadow -Wsign-compare -Wsign-conversion -Wstack-protector -Wstrict-overflow=5 -Wswitch -Wswitch-enum -Wsync-nand -Wvolatile-register-var -Wtrigraphs -Wtype-limits -Wuninitialized -Wunknown-pragmas -Wwrite-strings -Wpragmas -Wunreachable-code -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wno-return-type)

# Find pcl at the location specified by PCL_DIR
find_package(PCL 1.7 HINTS "${PCL_DIR}/share/pcl-1.7")

# Find OpenCV at the default location
find_package(OpenCV HINTS "${OPENCV_DIR}" REQUIRED)

# Only link with required opencv modules.
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui)

# Add the PCL and Eigen include dirs. 
# Necessary since the PCL_INCLUDE_DIR variable set bu find_package is broken.)
include_directories(${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR})
include_directories(${EIGEN_ROOT})

# Add source directory
aux_source_directory("./src" SRC_LIST)

# Add exectuteable
add_executable(${PROJECT_NAME} ${SRC_LIST})

# Link
target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PROJ4_LIBRARY} ${OpenCV_LIBS})
