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

## get sys vars
ARCH=`uname -m`
CORES=`grep -c processor /proc/cpuinfo`

						## dest base path
			TOOLS_PATH="$PWD"
						## paths for the tools
	    TOOLS_BIN_PATH="$TOOLS_PATH/bin"
	    TOOLS_INC_PATH="$TOOLS_PATH/include"
	    TOOLS_LIB_PATH="$TOOLS_PATH/lib"
	    TOOLS_SRC_PATH="$TOOLS_PATH/src"
	    TOOLS_LOG_PATH="$TOOLS_PATH/logs"
						## loacal dest paths
			  INC_PATH="/usr/local/include"
		if [ "$ARCH" = "i686" ]; then
			  LIB_PATH="/usr/lib"
		fi
		if [ "$ARCH" = "x86_64" ]; then
			  LIB_PATH="/usr/lib64"
		fi
						## source paths
		  BUNDLER_PATH="$TOOLS_SRC_PATH/bundler"
			 CMVS_PATH="$TOOLS_SRC_PATH/cmvs"
			 PMVS_PATH="$TOOLS_SRC_PATH/pmvs"
		  GRACLUS_PATH="$TOOLS_SRC_PATH/graclus"
		  CLAPACK_PATH="$TOOLS_SRC_PATH/clapack"
		   OPENCV_PATH="$TOOLS_SRC_PATH/openCv"
		   VLFEAT_PATH="$TOOLS_SRC_PATH/vlfeat"
		 PARALLEL_PATH="$TOOLS_SRC_PATH/parallel"
			  PSR_PATH="$TOOLS_SRC_PATH/PoissonRecon"
						## executables
		 EXTRACT_FOCAL="$TOOLS_BIN_PATH/extract_focal.pl"
			 MATCHKEYS="$TOOLS_BIN_PATH/KeyMatchFull"
			   BUNDLER="$TOOLS_BIN_PATH/bundler"
		   BUNDLE2PVMS="$TOOLS_BIN_PATH/Bundle2PMVS"
				  CMVS="$TOOLS_BIN_PATH/cmvs"
				  PMVS="$TOOLS_BIN_PATH/pmvs2"
			 GENOPTION="$TOOLS_BIN_PATH/genOption"
			    VLSIFT="$TOOLS_BIN_PATH/vlsift"
			  PARALLEL="$TOOLS_BIN_PATH/parallel"
				   PSR="$TOOLS_BIN_PATH/PoissonRecon"
	VLSIFT_TO_LOWESIFT="$TOOLS_BIN_PATH/convert_vlsift_to_lowesift.pl"

## prevents different (localized) output
LC_ALL=C

## removing old stuff
rm -rf `ls -1 | egrep -v '\.zip$|\.tgz$|\.bz2$|\.gz$|\.sh$|\.pl$' | xargs`

## create needed directories
mkdir -p "$TOOLS_BIN_PATH"
mkdir -p "$TOOLS_INC_PATH"
mkdir -p "$TOOLS_LIB_PATH"
mkdir -p "$TOOLS_SRC_PATH"
mkdir -p "$TOOLS_LOG_PATH"

## output sys info
echo "System info:" > "$TOOLS_LOG_PATH/sysinfo.txt"
uname -a > "$TOOLS_LOG_PATH/sysinfo.txt"

## install packages
echo
echo "  > installing required packages"

echo "    - updating"
sudo yum update -y > "$TOOLS_LOG_PATH/apt-get_get.log" 2>&1

echo "    - installing"
yum install -y \
	cmake gcc gcc-c++ compat-gcc-32 compat-gcc-32-c++ gcc-gfortran perl \
	curl wget \
	unzip \
	ImageMagick jhead \
	libjpeg-devel boost-devel gsl-devel libX11-devel libXext-devel lapack-devel blas-devel \
	zlib-devel \
	opencv-devel \
	> "$TOOLS_LOG_PATH/apt-get_install.log" 2>&1

echo "  < done - `date`"

## downloading sources
echo
echo "  > getting the sources"

git clone git://github.com/vlfeat/vlfeat.git --quiet > /dev/null 2>&1 &

## getting all archives if not already present; save them to .tmp and rename them after download
while read target source
do
	if [ ! -f "$target" ] ; then
		echo "    - getting $source"
		
		curl --progress-bar --insecure --location -o "$target.tmp" "$source"
		mv "$target.tmp" "$target"
		echo "    - finished $target"
		echo
	else
		echo "    - already downloaded $source"
	fi
done <<EOF
parallel.tar.bz2  http://ftp.gnu.org/gnu/parallel/parallel-20100922.tar.bz2
clapack.tgz	 http://www.netlib.org/clapack/clapack-3.2.1-CMAKE.tgz
bundler.zip	 http://phototour.cs.washington.edu/bundler/distr/bundler-v0.4-source.zip
graclus.tar.gz https://www.topoi.hu-berlin.de/graclus1.2.tar.gz
PoissonRecon.zip http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version2/PoissonRecon.zip
cmvs.tar.gz http://grail.cs.washington.edu/software/cmvs/cmvs-fix2.tar.gz
EOF

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

mv -f clapack-3.2.1-CMAKE	"$CLAPACK_PATH"
mv -f vlfeat				"$VLFEAT_PATH"
mv -f graclus1.2			"$GRACLUS_PATH"
mv -f bundler-v0.4-source	"$BUNDLER_PATH"
mv -f cmvs					"$CMVS_PATH"
mv -f parallel-20100922		"$PARALLEL_PATH"
mv -f PoissonRecon			"$PSR_PATH"

echo "    done - `date`"

# building
echo
echo "  - building"

sudo chown -R `id -u`:`id -g` *
sudo chmod -R 777 *

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
	make -j$CORES > "$TOOLS_LOG_PATH/parallel_2_build.log" 2>&1
	

	cp -f src/parallel "$TOOLS_BIN_PATH/"
	
echo "  < done - `date`"
echo

echo "  > clapack"
	cd "$CLAPACK_PATH"
	cp make.inc.example make.inc
	
	set +e
	echo "    - building clapack"
	make all -j$CORES > "$TOOLS_LOG_PATH/clapack_1_build.log" 2>&1
	set -e
	
	echo "    - installing clapack"
	make lapack_install > "$TOOLS_LOG_PATH/clapack_2_install.log" 2>&1

	cp -Rf INCLUDE "$INC_PATH/clapack"
	
echo "  < done - `date`"
echo

echo "  > vlfeat"
	cd "$VLFEAT_PATH"

	echo "    - cleaning vlfeat"
	make clean > "$TOOLS_LOG_PATH/vlfeat_1_clean.log" 2>&1

	echo "    - building vlfeat"
	make > "$TOOLS_LOG_PATH/vlfeat_2_build.log" 2>&1

	if [ "$ARCH" = "i686" ]; then
		cp -f "$VLFEAT_PATH/bin/glnx32/sift" "$TOOLS_BIN_PATH/vlsift"
		cp -f "$VLFEAT_PATH/bin/glnx32/libvl.so" "$TOOLS_LIB_PATH/"
	fi

	if [ "$ARCH" = "x86_64" ]; then
		cp -f "$VLFEAT_PATH/bin/glnxa64/sift" "$TOOLS_BIN_PATH/vlsift"
		cp -f "$VLFEAT_PATH/bin/glnxa64/libvl.so" "$TOOLS_LIB_PATH/"
	fi
echo "  < done - `date`"
echo

echo "  > cmvs"
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
		sed -i "$CMVS_PATH/program/main/Makefile" -e '24c\		-fopenmp -DNUMBITS=32 ${OPENMP_FLAG}'
	fi

	if [ "$ARCH" = "x86_64" ]; then
		sed -i "$CMVS_PATH/program/main/Makefile" -e "22c\CXXFLAGS_CMVS = -O2 -Wall -Wno-deprecated -DNUMBITS=64 \\\\"
		sed -i "$CMVS_PATH/program/main/Makefile" -e '24c\		-fopenmp -DNUMBITS=64 ${OPENMP_FLAG}'
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

echo "  > bundler"
	cd "$BUNDLER_PATH"

	echo "    - cleaning bundler"
	make clean > "$TOOLS_LOG_PATH/bundler_1_clean.log" 2>&1

	echo "    - building bundler"
	make -j $CORES > "$TOOLS_LOG_PATH/bundler_2_build.log" 2>&1

	cp -f "$BUNDLER_PATH/bin/Bundle2PMVS" "$BUNDLER_PATH/bin/Bundle2Vis" "$BUNDLER_PATH/bin/KeyMatchFull" "$BUNDLER_PATH/bin/bundler" "$BUNDLER_PATH/bin/RadialUndistort" "$TOOLS_BIN_PATH/"

	cp -f "$BUNDLER_PATH/lib/libANN_char.so" "$TOOLS_LIB_PATH/"
echo "  < done - `date`"
echo


cd "$TOOLS_PATH"

sudo install -o `id -u` -g `id -g` -m 644 -t "$LIB_PATH" lib/*.so
sudo ldconfig -v > "$TOOLS_LOG_PATH/ldconfig.log" 2>&1

sudo chown -R `id -u`:`id -g` *
sudo chmod -R 777 *

echo "  - script finished - `date`"

exit