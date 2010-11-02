#!/bin/bash

echo "created by Daniel Schwarz/daniel.schwarz@topoi.org"
echo "released under Creative Commons/CC-BY-NC"
echo "Attribution Non-Commercial"

ARCH=`uname -m`
TOOLS_PATH=$PWD
TOOLS_BIN_PATH=$TOOLS_PATH/bin

INC_PATH="/usr/include"
LIB_PATH="/usr/lib"
BIN_PATH="/usr/bin"

BUNDLER_PATH="$TOOLS_PATH/bundler"
CMVS_PATH="$TOOLS_PATH/cmvs"
PMVS_PATH="$TOOLS_PATH/pmvs"
SIFT_PATH="$TOOLS_PATH/lib/sift"
GRACLUS_PATH="$TOOLS_PATH/lib/graclus"
CLAPACK_PATH="$TOOLS_PATH/lib/clapack"
OPENCV_PATH="$TOOLS_PATH/lib/openCv"
VLFEAT_PATH="$TOOLS_PATH/lib/vlfeat"

echo
echo ---- installing required packages ----
echo

sudo apt-get update
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
	libboost-dev

echo
echo ---- getting the tools ----
echo


wget -O clapack.tgz	 http://www.netlib.org/clapack/clapack-3.2.1-CMAKE.tgz
wget -O bundler.zip	 http://phototour.cs.washington.edu/bundler/distr/bundler-v0.4-source.zip
wget -O sift.zip	 http://www.cs.ubc.ca/~lowe/keypoints/siftDemoV4.zip
wget -O graclus.tar.gz	 --no-check-certificate https://www.topoi.hu-berlin.de/graclus1.2.tar.gz
wget -O opencv.tar.bz2	 http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.1/OpenCV-2.1.0.tar.bz2/download
wget -O pmvs.tar.gz	 http://grail.cs.washington.edu/software/pmvs/pmvs-2-fix0.tar.gz
wget -O cmvs.tar.gz	 http://grail.cs.washington.edu/software/cmvs/cmvs-fix1.tar.gz

echo
echo ---- unzipping ----
echo

tar -xzf clapack.tgz& PID_CLAPACK=$!
unzip -q bundler.zip& PID_BUNDLER=$!
tar -xzf cmvs.tar.gz& PID_CMVS=$!
tar -xzf graclus.tar.gz& PID_GRACLUS=$!
unzip -q sift.zip& PID_SIFT=$!
tar -xf opencv.tar.bz2& PID_OPENCV=$!
tar -xzf pmvs.tar.gz& PID_PMVS=$!

git clone git://github.com/vlfeat/vlfeat.git& PID_VLFEAT=$!

wait $PID_CLAPACK
wait $PID_BUNDLER
wait $PID_PMVS
wait $PID_CMVS
wait $PID_GRACLUS
wait $PID_SIFT
wait $PID_OPENCV
wait $PID_VLFEAT

rm -f wget*
rm -f clapack.tgz
rm -f bundler.zip
rm -f pmvs.tar.gz
rm -f graclus.tar.gz
rm -f cmvs.tar.gz
rm -f sift.zip
rm -f opencv.tar.bz2

echo
echo ---- renaming ----
echo

mkdir -p $TOOLS_PATH/lib

mv -f clapack-3.2.1-CMAKE $CLAPACK_PATH
mv -f bundler-v0.4-source $BUNDLER_PATH
mv -f graclus1.2 $GRACLUS_PATH
mv -f siftDemoV4 $SIFT_PATH
mv -f cmvs $CMVS_PATH
mv -f OpenCV-2.1.0 $OPENCV_PATH
mv -f pmvs $PMVS_PATH
mv -f vlfeat $VLFEAT_PATH

sudo cp -R $CLAPACK_PATH/INCLUDE $INC_PATH/clapack

echo
echo ---- fixing ----
echo

sed -i $CMVS_PATH/program/main/genOption.cc -e "5c\#include <stdlib.h>\n" 
sed -i $CMVS_PATH/program/base/cmvs/bundle.cc -e "3c\#include <numeric>\n"

sed -i $CMVS_PATH/program/main/Makefile -e "10c\#Your INCLUDE path (e.g., -I\/usr\/include)" 
sed -i $CMVS_PATH/program/main/Makefile -e "11c\YOUR_INCLUDE_PATH =-I$INC_PATH" 
sed -i $CMVS_PATH/program/main/Makefile -e "13c\#Your metis directory (contains header files under graclus1.2/metisLib/)" 
sed -i $CMVS_PATH/program/main/Makefile -e "14c\YOUR_INCLUDE_METIS_PATH = -I$GRACLUS_PATH/metisLib/"
sed -i $CMVS_PATH/program/main/Makefile -e "16c\#Your LDLIBRARY path (e.g., -L/usr/lib)" 
sed -i $CMVS_PATH/program/main/Makefile -e "17c\YOUR_LDLIB_PATH = -L$LIB_PATH"

if [ "$ARCH" = "i686" ]; then
	sed -i $CMVS_PATH/program/main/Makefile -e "22c\CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=32 \\\\"
	sed -i $CMVS_PATH/program/main/Makefile -e '24c\		-fopenmp -DNUMBITS=32 ${OPENMP_FLAG}'
fi

if [ "$ARCH" = "x86_64" ]; then
	sed -i $CMVS_PATH/program/main/Makefile -e "22c\CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=64 \\\\"
	sed -i $CMVS_PATH/program/main/Makefile -e '24c\		-fopenmp -DNUMBITS=64 ${OPENMP_FLAG}'
fi

#sed -i $PMVS_PATH/program/main/Makefile -e "11c\#Your INCLUDE path (e.g., -I\/usr\/include)" 
#sed -i $PMVS_PATH/program/main/Makefile -e "12c\YOUR_INCLUDE_PATH =-I$INC_PATH" 
#sed -i $PMVS_PATH/program/main/Makefile -e "14c\#Your LDLIBRARY path (e.g., -L/usr/lib)" 
#sed -i $PMVS_PATH/program/main/Makefile -e "15c\YOUR_LDLIB_PATH = -L$LIB_PATH"

sed -i $BUNDLER_PATH/bin/extract_focal.pl -e '18c\    $JHEAD_EXE = "jhead";'
sed -i $BUNDLER_PATH/bin/ToSift.sh -e '36c\    echo "SIFT -o $key_file -x $d; gzip -f $key_file"'

if [ "$ARCH" = "i686" ]; then
	sed -i $GRACLUS_PATH/Makefile.in -e "11c\COPTIONS = -DNUMBITS=32"
fi

if [ "$ARCH" = "x86_64" ]; then
	sed -i $GRACLUS_PATH/Makefile.in -e "11c\COPTIONS = -DNUMBITS=64"
fi

echo
echo ---- building ----
echo

sudo chown -R $USER:$USER *
sudo chmod -R 755 *

cd $OPENCV_PATH
cmake .
sudo make
sudo make install

cd $SIFTFEAT_PATH
sudo make

cd $CLAPACK_PATH
cp make.inc.example make.inc
sudo make
sudo make lapack_install

cd $BUNDLER_PATH
rm -f bin/bundler bin/Bundle2PMVS bin/Bundle2Vis bin/bundler bin/jhead bin/jhead.exe bin/KeyMatchFull bin/libANN_char.so bin/RadialUndistort bin/zlib1.dll
sudo make clean
sudo make

cd $GRACLUS_PATH
sudo make
sudo cp lib* /usr/lib/

cd $PMVS_PATH/program/main
sudo make clean
sudo make depend
sudo make

cd $CMVS_PATH/program/main
sudo make clean
sudo make depend
sudo make

cd $VLFEAT_PATH/
sudo make clean
sudo make

cd $TOOLS_PATH

echo
echo ---- copying to dest loactions ----
echo

mkdir $TOOLS_BIN_PATH
cd $TOOLS_BIN_PATH
#wget --no-check-certificate  https://www.topoi.hu-berlin.de/run.sh
#sudo chmod a+x $TOOLS_BIN_PATH/run.sh

cp $SIFT_PATH/sift $TOOLS_BIN_PATH/
cp $VLFEAT_PATH/bin/glx/sift $TOOLS_BIN_PATH/vlsift

#cp $SIFTFEAT_PATH/bin/siftfeat $TOOLS_BIN_PATH/

cp $BUNDLER_PATH/bin/Bundle2PMVS $BUNDLER_PATH/bin/Bundle2Vis $BUNDLER_PATH/bin/KeyMatchFull $BUNDLER_PATH/bin/bundler $BUNDLER_PATH/bin/extract_focal.pl $BUNDLER_PATH/bin/RadialUndistort $TOOLS_BIN_PATH/

cp $CMVS_PATH/program/main/cmvs $CMVS_PATH/program/main/pmvs2 $CMVS_PATH/program/main/genOption $TOOLS_BIN_PATH/

sudo cp $VLFEAT_PATH/bin/glx/libvl.so $LIB_PATH/
sudo cp $BUNDLER_PATH/lib/libANN_char.so $LIB_PATH/

if [ "$ARCH" = "i686" ]; then
	cp $VLFEAT_PATH/bin/glx/sift $TOOLS_BIN_PATH/vlsift
	sudo cp $VLFEAT_PATH/bin/glx/libvl.so $LIB_PATH/
fi

if [ "$ARCH" = "x86_64" ]; then
	cp $VLFEAT_PATH/bin/a64/sift $TOOLS_BIN_PATH/vlsift
	sudo cp $VLFEAT_PATH/bin/a64/libvl.so $LIB_PATH/
fi

exit