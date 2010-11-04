#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo 
echo "     created by Daniel Schwarz/daniel.schwarz@topoi.org"
echo "     released under Creative Commons/CC-BY-NC"
echo "     Attribution Non-Commercial"
echo
echo "     if the script doesn't finish properly (i.e. it doesn't print \"script finished\" at the end)"
echo "     please email me the content of the logs folder"
echo
echo
echo "  - script started - `date`"

ARCH=`uname -m`
CORES=`ls -d /sys/devices/system/cpu/cpu[[:digit:]]* | wc -w`

# prevents different (localized) output
LC_ALL=C

## removing old stuff
rm -irf `ls -1 | egrep -v '\.zip$|\.tgz$|\.bz2$|\.gz$|\.sh$|^bin$' | xargs`
rm -irf `find bin | egrep -v '\.pl$|^bin$' | xargs`
#find . -maxdepth 0 \! -name \*.tgz \! -name \*.bz2 \! -name \*.gz \! -name \*.sh \! -name bin -delete
#find bin \! -name \*.pl \! -name bin -delete

## create needed directories
mkdir -p $TOOLS_BIN_PATH
mkdir -p $TOOLS_INC_PATH
mkdir -p $TOOLS_LIB_PATH
mkdir -p $TOOLS_SRC_PATH
mkdir -p $TOOLS_LOG_PATH

## output sys info
echo "System info:" > $TOOLS_LOG_PATH/sysinfo.txt
uname -a > $TOOLS_LOG_PATH/sysinfo.txt

## install packages
echo
echo "  - installing required packages"

sudo apt-get update --assume-yes > $TOOLS_LOG_PATH/apt-get_get.log 2>&1
sudo apt-get install --assume-yes --install-recommends \
	gcc g++	 gFortran cmake build-essential \
	imagemagick unzip wget \
	libzip-dev libjpeg-dev libtiff-dev libpng-dev libjasper-dev libann-dev \
	libavformat-dev ffmpeg python-opencv opencv-doc libcv-dev libcvaux-dev libhighgui-dev \
	libgsl0-dev libgsl0ldbl \
	libblas-dev libblas3gf \
	libhighgui-dev libcvaux-dev libcv-dev \
	liblapack-dev liblapack3gf \
	libx11-data libx11-dev libx11-6 \
	jhead \
	gtk2-engines doxygen \
	libpthread-stubs0 libpthread-stubs0-dev \
	libxext-dev libxext6 \
	curl pax \
	libboost-dev > $TOOLS_LOG_PATH/apt-get_install.log 2>&1

echo "    done - `date`"

## downloading sources
echo
echo "  - getting the sources"

if [ ! -f "parallel.tar.bz2" ]
then
	curl --location -o parallel.tar.bz2	 http://ftp.gnu.org/gnu/parallel/parallel-20100922.tar.bz2 > /dev/null 2>&1  & 
fi

if [ ! -f "clapack.tgz" ]
then
	curl --location -o clapack.tgz	 http://www.netlib.org/clapack/clapack-3.2.1-CMAKE.tgz > /dev/null 2>&1  & 
fi

if [ ! -f "bundler.zip" ]
then
	curl --location -o bundler.zip	 http://phototour.cs.washington.edu/bundler/distr/bundler-v0.4-source.zip > /dev/null 2>&1 & 
fi

if [ ! -f "graclus.tar.gz" ]
then
	curl --location -o graclus.tar.gz https://www.topoi.hu-berlin.de/graclus1.2.tar.gz > /dev/null 2>&1 & 
fi

if [ ! -f "opencv.tar.bz2" ]
then
	curl --location -o opencv.tar.bz2	 http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.1/OpenCV-2.1.0.tar.bz2/download > /dev/null 2>&1 &
fi

if [ ! -f "cmvs.tar.gz" ]
then
	curl --location -o cmvs.tar.gz	 http://grail.cs.washington.edu/software/cmvs/cmvs-fix1.tar.gz > /dev/null 2>&1 & 
fi

if [ ! -f "siftDemoV4.zip" ]
then
	curl --location -o siftDemoV4.zip	 http://www.cs.ubc.ca/~lowe/keypoints/siftDemoV4.zip > /dev/null 2>&1 & 
fi

if [ ! -f "PoissonRecon.zip" ]
then
	curl --location -o PoissonRecon.zip	 http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version2/PoissonRecon.zip > /dev/null 2>&1 & 
fi

git clone git://github.com/vlfeat/vlfeat.git --quiet > /dev/null 2>&1 & 

wait

echo "    done - `date`"

## unzipping sources
echo
echo "  - unzipping sources"

tar -xf opencv.tar.bz2 &
tar -xzf clapack.tgz &
tar -xzf graclus.tar.gz &
unzip -q bundler.zip &
unzip -q siftDemoV4.zip &
unzip -q PoissonRecon.zip &
tar -xzf cmvs.tar.gz &
tar -xf parallel.tar.bz2 &

wait
mv -f OpenCV-2.1.0 $OPENCV_PATH
mv -f clapack-3.2.1-CMAKE $CLAPACK_PATH
mv -f vlfeat $VLFEAT_PATH
mv -f graclus1.2 $GRACLUS_PATH
mv -f bundler-v0.4-source $BUNDLER_PATH
mv -f cmvs $CMVS_PATH
mv -f parallel-20100922 $PARALLEL_PATH
mv -f PoissonRecon $PSR_PATH
mv -f siftDemoV4 $SIFT_PATH

echo "    done - `date`"

# building
echo
echo "  - building (will take some time ...)"

sudo chown -R `id -u`:`id -g` *
sudo chmod -R 777 *

echo "  > poisson surface reconstruction "
	cd $PSR_PATH
	
	sed -i $PSR_PATH/Makefile -e "21c\BIN = ./"
		
	echo "    - building poisson surface reconstruction"
	sudo make -j$CORES > $TOOLS_LOG_PATH/poisson_1_build.log 2>&1
	
	cp -f $PSR_PATH/PoissonRecon $TOOLS_BIN_PATH/PoissonRecon
	
echo "  < done - `date`"
echo

echo "  > parallel"
	cd $PARALLEL_PATH
	
	echo "    - configuring parallel"
	./configure > $TOOLS_LOG_PATH/parallel_1_build.log 2>&1
	
	echo "    - building paralel"
	make -j$CORES > $TOOLS_LOG_PATH/parallel_2_build.log 2>&1
	

	cp -f src/parallel $TOOLS_BIN_PATH/
	
echo "  < done - `date`"
echo

echo "  > sift"
	cp $SIFT_PATH/sift $TOOLS_BIN_PATH/
	
echo "  < done - `date`"
echo

echo "  > clapack"
	cd $CLAPACK_PATH
	cp make.inc.example make.inc
	
	set +e
	echo "    - building clapack"
	sudo make all -j$CORES > $TOOLS_LOG_PATH/clapack_1_build.log 2>&1
	set -e
	
	echo "    - installing clapack"
	sudo make lapack_install > $TOOLS_LOG_PATH/clapack_2_install.log 2>&1

	cp -Rf INCLUDE $INC_PATH/clapack
	
echo "  < done - `date`"
echo

echo "  > opencv"
	mkdir -p $OPENCV_PATH/release
	cd $OPENCV_PATH/release
	
	echo "    - generating makefiles for opencv"
	(sudo cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$TOOLS_PATH ..) > $TOOLS_LOG_PATH/opencv_1_cmake.log 2>&1
	
	echo "    - building opencv"
	sudo make -j$CORES > $TOOLS_LOG_PATH/opencv_3_build.log 2>&1
	
	echo "    - installing opencv"
	sudo make install > $TOOLS_LOG_PATH/opencv_4_install.log 2>&1
echo "  < done - `date`"
echo 

echo "  > vlfeat"
	cd $VLFEAT_PATH

	echo "    - cleaning vlfeat"
	sudo make clean > $TOOLS_LOG_PATH/vlfeat_1_clean.log 2>&1

	echo "    - building vlfeat"
	sudo make > $TOOLS_LOG_PATH/vlfeat_2_build.log 2>&1

	if [ "$ARCH" = "i686" ]; then
		cp -f $VLFEAT_PATH/bin/glx/sift $TOOLS_BIN_PATH/vlsift
		sudo cp -f $VLFEAT_PATH/bin/glx/libvl.so $TOOLS_LIB_PATH/
	fi

	if [ "$ARCH" = "x86_64" ]; then
		cp -f $VLFEAT_PATH/bin/a64/sift $TOOLS_BIN_PATH/vlsift
		sudo cp -f $VLFEAT_PATH/bin/a64/libvl.so $TOOLS_LIB_PATH/
	fi
echo "  < done - `date`"
echo

echo "  > graclus"
	cd $GRACLUS_PATH

	if [ "$ARCH" = "i686" ]; then
		sed -i $GRACLUS_PATH/Makefile.in -e "11c\COPTIONS = -DNUMBITS=32"
	fi

	if [ "$ARCH" = "x86_64" ]; then
		sed -i $GRACLUS_PATH/Makefile.in -e "11c\COPTIONS = -DNUMBITS=64"
	fi
	
	echo "    - cleaning graclus"
	sudo make clean > $TOOLS_LOG_PATH/graclus_1_clean.log 2>&1

	echo "    - building graclus"
	sudo make -j$CORES > $TOOLS_LOG_PATH/graclus_2_build.log 2>&1
   
   mkdir $TOOLS_INC_PATH/metisLib
   cp -f $GRACLUS_PATH/metisLib/*.h $TOOLS_INC_PATH/metisLib/

	sudo cp -f lib* $TOOLS_LIB_PATH/
echo "  < done - `date`"
echo

echo "  > bundler"
	cd $BUNDLER_PATH

	echo "    - cleaning bundler"
	sudo make clean > $TOOLS_LOG_PATH/bundler_1_clean.log 2>&1

	echo "    - building bundler"
	sudo make -j $CORES > $TOOLS_LOG_PATH/bundler_2_build.log 2>&1

	cp -f $BUNDLER_PATH/bin/Bundle2PMVS $BUNDLER_PATH/bin/Bundle2Vis $BUNDLER_PATH/bin/KeyMatchFull $BUNDLER_PATH/bin/bundler $BUNDLER_PATH/bin/extract_focal.pl $BUNDLER_PATH/bin/RadialUndistort $TOOLS_BIN_PATH/

	sudo cp -f $BUNDLER_PATH/lib/libANN_char.so $TOOLS_LIB_PATH/

	sed -i $TOOLS_BIN_PATH/extract_focal.pl -e '18c\    $JHEAD_EXE = "jhead";'
echo "  < done - `date`"
echo

echo "  > cmvs"
	cd $CMVS_PATH/program/main

	sed -i $CMVS_PATH/program/main/genOption.cc -e "5c\#include <stdlib.h>\n" 
	sed -i $CMVS_PATH/program/base/cmvs/bundle.cc -e "3c\#include <numeric>\n"
	
	sed -i $CMVS_PATH/program/main/Makefile -e "10c\#Your INCLUDE path (e.g., -I\/usr\/include)" 
	sed -i $CMVS_PATH/program/main/Makefile -e "11c\YOUR_INCLUDE_PATH =-I$INC_PATH -I$TOOLS_INC_PATH" 
	sed -i $CMVS_PATH/program/main/Makefile -e "13c\#Your metis directory (contains header files under graclus1.2/metisLib/)" 
	sed -i $CMVS_PATH/program/main/Makefile -e "14c\YOUR_INCLUDE_METIS_PATH = -I$TOOLS_INC_PATH/metisLib/"
	sed -i $CMVS_PATH/program/main/Makefile -e "16c\#Your LDLIBRARY path (e.g., -L/usr/lib)" 
	sed -i $CMVS_PATH/program/main/Makefile -e "17c\YOUR_LDLIB_PATH = -L$LIB_PATH -L$TOOLS_LIB_PATH"
	
	if [ "$ARCH" = "i686" ]; then
		sed -i $CMVS_PATH/program/main/Makefile -e "22c\CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=32 \\\\"
		sed -i $CMVS_PATH/program/main/Makefile -e '24c\		-fopenmp -DNUMBITS=32 ${OPENMP_FLAG}'
	fi

	if [ "$ARCH" = "x86_64" ]; then
		sed -i $CMVS_PATH/program/main/Makefile -e "22c\CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=64 \\\\"
		sed -i $CMVS_PATH/program/main/Makefile -e '24c\		-fopenmp -DNUMBITS=64 ${OPENMP_FLAG}'
	fi

	echo "    - cleaning cmvs"
	sudo make clean > $TOOLS_LOG_PATH/cmvs_1_clean.log 2>&1

	echo "    - building cmvs"
	sudo make -j$CORES > $TOOLS_LOG_PATH/cmvs_2_build.log 2>&1

	echo "    - make depend cmvs"
	sudo make depend > $TOOLS_LOG_PATH/cmvs_3_depend.log 2>&1

	cp -f $CMVS_PATH/program/main/cmvs $CMVS_PATH/program/main/pmvs2 $CMVS_PATH/program/main/genOption $TOOLS_BIN_PATH/
	cp -f $CMVS_PATH/program/main/*so* $TOOLS_LIB_PATH/
echo "  < done - `date`"
echo

cd $TOOLS_PATH

cp -f $TOOLS_LIB_PATH/*.so* $LIB_PATH/
ldconfig -v > $TOOLS_LOG_PATH/ldconfig.log 2>&1

echo "  - script finished - `date`"

exit