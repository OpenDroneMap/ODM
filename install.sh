#!/bin/bash

set -o nounset
set -o errexit

echo 
echo "     created by Daniel Schwarz/daniel.schwarz@topoi.org"
echo "     released under Creative Commons/CC-BY"
echo "     Attribution"
echo
echo "     if the script doesn't finish properly"
echo "     (i.e. it doesn't print \"script finished\" at the end)"
echo "     please email me the content of the logs folder"
echo
echo
echo "  - script started - `date`"

            ## dest base path
          TOOLS_PATH="$PWD"

            ## paths for the tools
      TOOLS_BIN_PATH="$TOOLS_PATH/bin"
      TOOLS_INC_PATH="$TOOLS_PATH/include"
      TOOLS_LIB_PATH="$TOOLS_PATH/lib"
        TOOLS_SRC_PATH="$TOOLS_PATH/src"
      TOOLS_LOG_PATH="$TOOLS_PATH/logs"
  TOOLS_PATCHED_PATH="$TOOLS_PATH/patched_files"

            ## loacal dest paths
		        LIB_PATH="/usr/local/lib"
		        INC_PATH="/usr/local/include"
		
            ## source paths
	      BUNDLER_PATH="$TOOLS_SRC_PATH/bundler"
	         CMVS_PATH="$TOOLS_SRC_PATH/cmvs"
	         PMVS_PATH="$TOOLS_SRC_PATH/pmvs"
	      CLAPACK_PATH="$TOOLS_SRC_PATH/clapack"
	       VLFEAT_PATH="$TOOLS_SRC_PATH/vlfeat"
	     PARALLEL_PATH="$TOOLS_SRC_PATH/parallel"
	          PSR_PATH="$TOOLS_SRC_PATH/PoissonRecon"
        GRACLUS_PATH="$TOOLS_SRC_PATH/graclus"
          CERES_PATH="$TOOLS_SRC_PATH/ceres-solver"

                  PCL_PATH="$TOOLS_SRC_PATH/pcl"
             LASTOOLS_PATH="$TOOLS_SRC_PATH/lastools"
          ODM_MESHING_PATH="$TOOLS_SRC_PATH/odm_meshing"
        ODM_TEXTURING_PATH="$TOOLS_SRC_PATH/odm_texturing"
       ODM_ORTHOPHOTO_PATH="$TOOLS_SRC_PATH/odm_orthophoto"
      ODM_EXTRACT_UTM_PATH="$TOOLS_SRC_PATH/odm_extract_utm"
           ODM_GEOREF_PATH="$TOOLS_SRC_PATH/odm_georef"

         OPENGV_PATH="$TOOLS_SRC_PATH/opengv"
        OPENSFM_PATH="$TOOLS_SRC_PATH/OpenSfM"

            ## executables
	     EXTRACT_FOCAL="$TOOLS_BIN_PATH/extract_focal.pl"
	         MATCHKEYS="$TOOLS_BIN_PATH/KeyMatch"
	     MATCHKEYSFULL="$TOOLS_BIN_PATH/KeyMatchFull"
	           BUNDLER="$TOOLS_BIN_PATH/bundler"
	       BUNDLE2PVMS="$TOOLS_BIN_PATH/Bundle2PMVS"
	              CMVS="$TOOLS_BIN_PATH/cmvs"
	              PMVS="$TOOLS_BIN_PATH/pmvs2"
	         GENOPTION="$TOOLS_BIN_PATH/genOption"
	            VLSIFT="$TOOLS_BIN_PATH/vlsift"
	          PARALLEL="$TOOLS_BIN_PATH/parallel"
	               PSR="$TOOLS_BIN_PATH/PoissonRecon"
	VLSIFT_TO_LOWESIFT="$TOOLS_BIN_PATH/convert_vlsift_to_lowesift.pl"

               ODM_MESHING="$TOOLS_BIN_PATH/odm_meshing"
             ODM_TEXTURING="$TOOLS_BIN_PATH/odm_texturing"
            ODM_ORTHOPHOTO="$TOOLS_BIN_PATH/odm_orthophoto"
           ODM_EXTRACT_UTM="$TOOLS_BIN_PATH/odm_extract_utm"
                ODM_GEOREF="$TOOLS_BIN_PATH/odm_georef"

## get sys vars
ARCH=`uname -m`
CORES=`grep -c processor /proc/cpuinfo`

## prevents different (localized) output
LC_ALL=C

## removing old stuff
sudo rm -Rf "$TOOLS_BIN_PATH"
sudo rm -Rf "$TOOLS_INC_PATH"
sudo rm -Rf "$TOOLS_LIB_PATH"
sudo rm -Rf "$TOOLS_SRC_PATH"
sudo rm -Rf "$TOOLS_LOG_PATH"

## create needed directories
mkdir -p "$TOOLS_BIN_PATH"
mkdir -p "$TOOLS_INC_PATH"
mkdir -p "$TOOLS_LIB_PATH"
mkdir -p "$TOOLS_SRC_PATH"
mkdir -p "$TOOLS_LOG_PATH"

## Copy meshing and texturing to src folder
cp -rf "odm_meshing" "$TOOLS_SRC_PATH/"
cp -rf "odm_texturing" "$TOOLS_SRC_PATH/"
cp -rf "odm_orthophoto" "$TOOLS_SRC_PATH/"
cp -rf "odm_extract_utm" "$TOOLS_SRC_PATH/"
cp -rf "odm_georef" "$TOOLS_SRC_PATH/"

## output sys info
echo "System info:" > "$TOOLS_LOG_PATH/sysinfo.txt"
uname -a > "$TOOLS_LOG_PATH/sysinfo.txt"

## install packages
echo
echo "  > installing required packages"

echo "    - updating"
sudo apt-get update --assume-yes > "$TOOLS_LOG_PATH/apt-get_get.log" 2>&1

echo "    - installing"
if [[ `lsb_release -rs` == "12.04" ]];
then
sudo apt-get install --assume-yes --install-recommends \
  build-essential cmake g++ gcc gFortran perl git autoconf \
  curl wget \
  unzip \
  imagemagick jhead proj-bin libproj-dev\
  libjpeg-dev libboost1.48-all-dev libgsl0-dev libx11-dev libxext-dev liblapack-dev \
  libeigen3-dev libflann-dev libvtk5-dev libqhull-dev libusb-1.0-0-dev\
  libzip-dev \
  libswitch-perl libjson-perl \
  libcv-dev libcvaux-dev libopencv-dev \
  gdal-bin \
  exiv2 \
  libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev \
  > "$TOOLS_LOG_PATH/apt-get_install.log" 2>&1
else
sudo apt-get install --assume-yes --install-recommends \
  build-essential cmake g++ gcc gFortran perl git autoconf \
  curl wget \
  unzip \
  imagemagick jhead proj-bin libproj-dev\
  libjpeg-dev libboost-all-dev libgsl0-dev libx11-dev libxext-dev liblapack-dev \
  libeigen3-dev libflann-dev libvtk5-dev libqhull-dev libusb-1.0-0-dev\
  libjson-perl \
  libzip-dev \
  libswitch-perl \
  libcv-dev libcvaux-dev libopencv-dev \
  libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev \
  python-dev python-pip libboost-python-dev \
  python-numpy-dev python-scipy python-yaml \
  python-opencv python-pyexiv2 \
  gdal-bin \
  exiv2 \
  > "$TOOLS_LOG_PATH/apt-get_install.log" 2>&1
fi

sudo pip install networkx exifread xmltodict

echo "    - installing git submodules"
git submodule init
git submodule update

echo "  < done - `date`"

## downloading sources
echo
echo "  > getting the sources"

## Reconstruct CMVS tar.gz from pieces...
cat cmvs.tar.gz.part-?? > cmvs.tar.gz

## getting all archives if not already present; save them to .tmp and rename them after download
while read target source
do
  if [ ! -f "$target" ] ; then
    echo "    - getting $source"
    
    curl --progress-bar --location -o "$target.tmp" "$source"
    mv "$target.tmp" "$target"
    echo "    - finished $target"
    echo
  else
    echo "    - already downloaded $source"
  fi
done <<EOF
parallel.tar.bz2  http://ftp.gnu.org/gnu/parallel/parallel-20141022.tar.bz2
clapack.tgz  http://www.netlib.org/clapack/clapack-3.2.1-CMAKE.tgz
PoissonRecon.zip http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version2/PoissonRecon.zip
vlfeat.tar.gz http://www.vlfeat.org/download/vlfeat-0.9.13-bin.tar.gz
cmvs.tar.gz http://www.di.ens.fr/cmvs/cmvs-fix2.tar.gz
graclus.tar.gz http://smathermather.github.io/BundlerTools/patched_files/src/graclus/graclus1.2.tar.gz
pcl.tar.gz https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz
ceres-solver.tar.gz http://ceres-solver.org/ceres-solver-1.10.0.tar.gz
LAStools.zip http://lastools.org/download/LAStools.zip
EOF
git clone  https://github.com/paulinus/opengv.git $OPENGV_PATH
git clone  https://github.com/mapillary/OpenSfM.git $OPENSFM_PATH

echo "  < done - `date`"

## unzipping sources
echo
echo "  - unzipping sources"

for i in *.tar.bz2 ; do
  tar xjf "$i" > "$TOOLS_LOG_PATH/extract_$i.log" 2>&1 & 
done
for i in *.tgz *.tar.gz ; do
  tar xzf "$i" > "$TOOLS_LOG_PATH/extract_$i.log" 2>&1 &
done
for i in *.zip ; do
  unzip "$i" > "$TOOLS_LOG_PATH/extract_$i.log" 2>&1 &
done

wait

mv -f graclus1.2          "$GRACLUS_PATH"
mv -f clapack-3.2.1-CMAKE "$CLAPACK_PATH"
mv -f vlfeat-0.9.13       "$VLFEAT_PATH"
mv -f parallel-20141022   "$PARALLEL_PATH"
mv -f PoissonRecon        "$PSR_PATH"
mv -f cmvs                "$CMVS_PATH"
mv -f pcl-pcl-1.7.2       "$PCL_PATH"
mv -f ceres-solver-1.10.0 "$CERES_PATH"
mv -f LAStools            "$LASTOOLS_PATH"


echo "  < done - `date`"


## copying patches
echo
echo "  - copying patches"
echo

for file in `find $TOOLS_PATCHED_PATH -type f -print` ; do
  cp $file $TOOLS_PATH/${file/$TOOLS_PATCHED_PATH/.}
done

echo "  < done - `date`"


# building
echo
echo "  - building"
echo

sudo chown -R `id -u`:`id -g` *
#sudo chmod -R 777 *


echo "  > graclus"
	cd "$GRACLUS_PATH"

	if [ "$ARCH" = "i686" ]; then
		sed -i "$GRACLUS_PATH/Makefile.in" -e "11c\COPTIONS = -DNUMBITS=32"
	fi

	if [ "$ARCH" = "x86_64" ]; then
		sed -i "$GRACLUS_PATH/Makefile.in" -e "11c\COPTIONS = -DNUMBITS=64"
	fi
	
	echo "    - cleaning graclus"
	make clean > "$TOOLS_LOG_PATH/graclus_1_clean.log" 2>&1

	echo "    - building graclus"
	make -j$CORES > "$TOOLS_LOG_PATH/graclus_2_build.log" 2>&1
   
	mkdir "$TOOLS_INC_PATH/metisLib"
	cp -f "$GRACLUS_PATH/metisLib/"*.h "$TOOLS_INC_PATH/metisLib/"

	cp -f lib* "$TOOLS_LIB_PATH/"
echo "  < done - `date`"
echo

echo "  > poisson surface reconstruction "
  cd "$PSR_PATH"
  
  sed -i "$PSR_PATH/Makefile" -e "21c\BIN = ./"
    
  echo "    - building poisson surface reconstruction"
  make -j$CORES > "$TOOLS_LOG_PATH/poisson_1_build.log" 2>&1
  
  cp -f "$PSR_PATH/PoissonRecon" "$TOOLS_BIN_PATH/PoissonRecon"
  
echo "  < done - `date`"
echo


echo "  > parallel"
  cd "$PARALLEL_PATH"
  
  echo "    - configuring parallel"
  ./configure > "$TOOLS_LOG_PATH/parallel_1_build.log" 2>&1
  
  echo "    - building paralel"
  make -j$CORES> "$TOOLS_LOG_PATH/parallel_2_build.log" 2>&1
  
  cp -f src/parallel "$TOOLS_BIN_PATH/"
  
echo "  < done - `date`"
echo


echo "  > clapack"
  cd "$CLAPACK_PATH"
  cp make.inc.example make.inc
  
  set +e
  echo "    - building clapack"
  make -j$CORES all > "$TOOLS_LOG_PATH/clapack_1_build.log" 2>&1
  set -e
  
  echo "    - installing clapack"
  make -j$CORES lapack_install > "$TOOLS_LOG_PATH/clapack_2_install.log" 2>&1

  sudo cp -Rf INCLUDE "$INC_PATH/clapack"
  
echo "  < done - `date`"
echo


echo "  > vlfeat"
  cd "$VLFEAT_PATH"
  
  echo "    - installing vlfeat"
  
  if [ "$ARCH" = "i686" ]; then
    cp -f "$VLFEAT_PATH/bin/glnx86/sift" "$TOOLS_BIN_PATH/vlsift"
    cp -f "$VLFEAT_PATH/bin/glnx86/libvl.so" "$TOOLS_LIB_PATH/"
  fi

  if [ "$ARCH" = "x86_64" ]; then
    cp -f "$VLFEAT_PATH/bin/glnxa64/sift" "$TOOLS_BIN_PATH/vlsift"
    cp -f "$VLFEAT_PATH/bin/glnxa64/libvl.so" "$TOOLS_LIB_PATH/"
  fi
echo "  < done - `date`"
echo

echo "  > LAStools"
  cd "$LASTOOLS_PATH"
  
  echo "    - installing LAStools"
  
  make -j$CORES > "$TOOLS_LOG_PATH/lastools_1_build.log" 2>&1

  if [ "$ARCH" = "i686" ]; then
    cp -f "$LASTOOLS_PATH/bin/txt2las" "$TOOLS_BIN_PATH/txt2las"
  fi

  if [ "$ARCH" = "x86_64" ]; then
    cp -f "$LASTOOLS_PATH/bin/txt2las" "$TOOLS_BIN_PATH/txt2las"
  fi
echo "  < done - `date`"
echo


echo "  > cmvs/pmvs"
  cd "$CMVS_PATH/program/main"

  sed -i "$CMVS_PATH/program/main/genOption.cc" -e "5c\#include <stdlib.h>\n" 
  sed -i "$CMVS_PATH/program/base/cmvs/bundle.cc" -e "3c\#include <numeric>\n"

  sed -i "$CMVS_PATH/program/main/Makefile" -e "10c\#Your INCLUDE path (e.g., -I\/usr\/include)" 
  sed -i "$CMVS_PATH/program/main/Makefile" -e "11c\YOUR_INCLUDE_PATH =-I$INC_PATH -I$TOOLS_INC_PATH" 
  sed -i "$CMVS_PATH/program/main/Makefile" -e "13c\#Your metis directory (contains header files under graclus1.2/metisLib/)" 
  sed -i "$CMVS_PATH/program/main/Makefile" -e "14c\YOUR_INCLUDE_METIS_PATH = -I$TOOLS_INC_PATH/metisLib/"
  sed -i "$CMVS_PATH/program/main/Makefile" -e "16c\#Your LDLIBRARY path (e.g., -L/usr/lib)" 
  sed -i "$CMVS_PATH/program/main/Makefile" -e "17c\YOUR_LDLIB_PATH = -L$LIB_PATH -L$TOOLS_LIB_PATH"

  if [ "$ARCH" = "i686" ]; then
    sed -i "$CMVS_PATH/program/main/Makefile" -e "22c\CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=32 \\\\"
    sed -i "$CMVS_PATH/program/main/Makefile" -e '24c\    -fopenmp -DNUMBITS=32 ${OPENMP_FLAG}'
  fi

  if [ "$ARCH" = "x86_64" ]; then
    sed -i "$CMVS_PATH/program/main/Makefile" -e "22c\CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=64 \\\\"
    sed -i "$CMVS_PATH/program/main/Makefile" -e '24c\    -fopenmp -DNUMBITS=64 ${OPENMP_FLAG}'
  fi

  echo "    - cleaning cmvs"
  make clean > "$TOOLS_LOG_PATH/cmvs_1_clean.log" 2>&1

  echo "    - building cmvs"
  make -j$CORES > "$TOOLS_LOG_PATH/cmvs_2_build.log" 2>&1

  echo "    - make depend cmvs"
  sudo make depend > "$TOOLS_LOG_PATH/cmvs_3_depend.log" 2>&1

  cp -f "$CMVS_PATH/program/main/cmvs" "$CMVS_PATH/program/main/pmvs2" "$CMVS_PATH/program/main/genOption" "$TOOLS_BIN_PATH/"
  cp -f "$CMVS_PATH/program/main/"*so* "$TOOLS_LIB_PATH/"
echo "  < done - `date`"
echo

echo "  > ceres"
  cd "$CERES_PATH"

  echo "    - configuring ceres"
  mkdir -p build
  cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=$TOOLS_PATH \
           -DCMAKE_C_FLAGS=-fPIC -DCMAKE_CXX_FLAGS=-fPIC \
           -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF  > "$TOOLS_LOG_PATH/ceres_1_config.log" 2>&1

  echo "    - building ceres"
  make -j$CORES install > "$TOOLS_LOG_PATH/ceres_1_build.log" 2>&1

echo "  < done - `date`"
echo

echo "  > bundler"
  cd "$BUNDLER_PATH"

  echo "    - cleaning bundler"
  make clean > "$TOOLS_LOG_PATH/bundler_1_clean.log" 2>&1

  echo "    - building bundler"
  make -j$CORES  > "$TOOLS_LOG_PATH/bundler_2_build.log" 2>&1

  ln -s "$BUNDLER_PATH/bin/Bundle2PMVS" "$BUNDLER_PATH/bin/Bundle2Vis" "$BUNDLER_PATH/bin/KeyMatchFull" "$BUNDLER_PATH/bin/KeyMatch" "$BUNDLER_PATH/bin/bundler" "$BUNDLER_PATH/bin/RadialUndistort" "$TOOLS_BIN_PATH/"

  ln -s "$BUNDLER_PATH/lib/libANN_char.so" "$TOOLS_LIB_PATH/"
echo "  < done - `date`"
echo

echo "  > pcl "
	#cd "$PCL_PATH"
	
	#Install pcl dependencies using the default package manager.
	#sudo apt-get install libeigen3-dev libflann-dev libvtk5-dev libqhull-dev

	#install the required boost version.
	#sudo apt-get install libboost1.48-all-dev

	mkdir -p "pcl"
	mkdir -p "$TOOLS_LIB_PATH/pcl"
	mkdir -p "$PCL_PATH/pcl_tmp"
	mkdir -p "$PCL_PATH/pcl_build"

	#mv -f "pcl-pcl-1.7.2" "$PCL_PATH/pcl_tmp"

	cd "$PCL_PATH/pcl_build"

	echo "    - configuring pcl"
  	
	cmake .. -DCMAKE_INSTALL_PREFIX="$TOOLS_LIB_PATH/pcl" -DCMAKE_BUILD_TYPE=Release -DPCL_VERBOSITY_LEVEL=Error -DBUILD_features=OFF -DBUILD_filters=OFF -DBUILD_geometry=OFF -DBUILD_keypoints=OFF -DBUILD_outofcore=OFF -DBUILD_people=OFF -DBUILD_recognition=OFF -DBUILD_registration=OFF -DBUILD_sample_consensus=OFF -DBUILD_segmentation=OFF -DBUILD_features=OFF -DBUILD_surface_on_nurbs=OFF -DBUILD_tools=OFF -DBUILD_tracking=OFF -DBUILD_visualization=OFF -DWITH_QT=OFF -DBUILD_OPENNI=OFF -DBUILD_OPENNI2=OFF -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_FZAPI=OFF -DWITH_LIBUSB=OFF -DWITH_PCAP=OFF -DWITH_PXCAPI=OFF > "$TOOLS_LOG_PATH/pcl_1_build.log" 2>&1
  
	echo "    - building and installing pcl"
	make install > "$TOOLS_LOG_PATH/pcl_2_build.log" 2>&1

echo "  < done - `date`"
echo

echo "  > meshing "
	cd "$ODM_MESHING_PATH"
	
	echo "    - configuring odm_meshing"
	cmake . -DPCL_DIR="$TOOLS_LIB_PATH/pcl" > "$TOOLS_LOG_PATH/odm_meshing_1_build.log" 2>&1
	
	echo "    - building odm_meshing"
	make -j$CORES > "$TOOLS_LOG_PATH/odm_meshing_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_meshing" "$TOOLS_BIN_PATH/odm_meshing"	
	
echo "  < done - `date`"
echo

echo "  > texturing "
	cd "$ODM_TEXTURING_PATH"
	
	echo "    - configuring odm_texturing"
	cmake . -DPCL_DIR="$TOOLS_LIB_PATH/pcl" > "$TOOLS_LOG_PATH/odm_texturing_1_build.log" 2>&1
	
	echo "    - building odm_texturing"
	make -j$CORES > "$TOOLS_LOG_PATH/odm_texturing_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_texturing" "$TOOLS_BIN_PATH/odm_texturing"	
	
echo "  < done - `date`"
echo

echo "  > extract_utm "
	cd "$ODM_EXTRACT_UTM_PATH"
	
	echo "    - configuring odm_extract_utm"
	cmake . > "$TOOLS_LOG_PATH/odm_extract_utm_1_build.log" 2>&1
	
	echo "    - building odm_extract_utm"
	make -j$CORES > "$TOOLS_LOG_PATH/odm_extract_utm_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_extract_utm" "$TOOLS_BIN_PATH/odm_extract_utm"	
	
echo "  < done - `date`"
echo

echo "  > georef "
	cd "$ODM_GEOREF_PATH"
	
	echo "    - configuring odm_georef"
	cmake . -DPCL_DIR="$TOOLS_LIB_PATH/pcl" > "$TOOLS_LOG_PATH/odm_georef_1_build.log" 2>&1
	
	echo "    - building odm_georef"
	make -j$CORES > "$TOOLS_LOG_PATH/odm_georef_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_georef" "$TOOLS_BIN_PATH/odm_georef"	
	
echo "  < done - `date`"
echo

echo "  > orthophoto "
	cd "$ODM_ORTHOPHOTO_PATH"
	
	echo "    - configuring odm_orthophoto"
	cmake . -DPCL_DIR="$TOOLS_LIB_PATH/pcl" > "$TOOLS_LOG_PATH/odm_orthophoto_1_build.log" 2>&1
	
	echo "    - building odm_orthophoto"
	make -j$CORES > "$TOOLS_LOG_PATH/odm_orthophoto_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_orthophoto" "$TOOLS_BIN_PATH/odm_orthophoto"	
	
echo "  < done - `date`"
echo

echo "  > OpenGV"
  cd "$OPENGV_PATH"

  echo "    - configuring opengv"
  git checkout python-wrapper
  mkdir -p build
  cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=$TOOLS_PATH -DBUILD_TESTS=OFF -DBUILD_PYTHON=ON > "$TOOLS_LOG_PATH/opengv_1_build.log" 2>&1
  echo "    - building opengv"
  make install > "$TOOLS_LOG_PATH/opengv_2_build.log" 2>&1

echo "  < done - `date`"
echo

echo "  > OpenSfM"
  cd "$OPENSFM_PATH"

  echo "    - configuring opensfm"
  git checkout odm-2
  echo "    - building opensfm"
  CERES_ROOT_DIR=$TOOLS_PATH python setup.py build > "$TOOLS_LOG_PATH/opensfm_1_build.log" 2>&1

echo "  < done - `date`"
echo


cd "$TOOLS_PATH"

sudo install -o `id -u` -g `id -g` -m 644 -t "$LIB_PATH" lib/*.so
sudo ldconfig -v > "$TOOLS_LOG_PATH/ldconfig.log" 2>&1

sudo chown -R `id -u`:`id -g` *
#sudo chmod -R 777 *
#sudo chmod 700 run.pl
echo "  - script finished - `date`"
exit
