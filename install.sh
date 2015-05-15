#!/bin/bash

# set debug mode
if [ -n "${DEBUG}" ]
then
    set -h
    set -x
    set -u
    set -e

fi;

trap "exit 1" TERM

# default flag values
CLEAN_BUILD_DIR=0
INSTALL_PACKAGES=0
TOOLS_PATH=$(pwd)
DEST_PATH='/usr/local'

GETOPTS="d:l:cih?"

function die(){
    echo "${1}"
    kill -s TERM $$
}

function usage(){
        set +x
        echo
        printf "Usage: %s [-d INSTALL_DIR] [-c] [-i] \n" $0
        echo " -d INSTALL_DIR install data to location"
        echo " -c clean installation directory first"
        echo " -i install system packages"
        echo " -l install results in provided dir"
        echo
}

function do_install(){
    # src can be glob
    src="${1}"
    # construct destination including $DEST_PATH
    dest="$DEST_PATH/${2}"

    set +u
    # file mode
    if [ -n "${3}" ]
    then 
        fmode="${3}"
    else
        fmode=755
    fi;
    set -u
    mkdir -p $dest
    
    install -v -o $(id -u) -g $(id -g) -m $fmode -C -t $dest $src || die "Installation of $src failed"

}

getopts_passed=0
while getopts $GETOPTS name
do
    case $name in
    c)  
        getopts_passed=1
        CLEAN_BUILD_DIR=1
        ;;
    i)  
        getopts_passed=1
        INSTALL_PACKAGES=1
        ;;
    d)  
        if [ "$OPTARG" ]
        then
            getopts_passed=1
            TOOLS_PATH="$OPTARG"
        fi
        ;; 
    \?)
        usage
        exit 1
        ;;
    h)
        usage
        exit 1
        ;;
    l)
        if [ "$OPTARG" ]
        then
            getopts_passed=1
            DEST_PATH="$OPTARG"
        fi
        ;; 
        
    esac
done

echo "creating destination ${TOOLS_PATH}"
mkdir -p $TOOLS_PATH
pushd $TOOLS_PATH > /dev/null
TOOLS_PATH=$(pwd)
popd > /dev/null
echo "Absolute path: ${TOOLS_PATH}"

echo "creating destination installation path: ${DEST_PATH}"
mkdir -p $DEST_PATH
pushd $DEST_PATH > /dev/null
DEST_PATH=$(pwd)
popd > /dev/null
echo "Absolute path: ${DEST_PATH}"



if [ $getopts_passed -ne 1 ]
then
    usage
    exit 1
fi;

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

if [ "$INSTALL_PACKAGES" -ne 0 ] ; then echo " * Will install packages" ; fi;
if [ "$CLEAN_BUILD_DIR" -ne 0 ] ; then echo " * Will clean install dir first" ; fi;

echo " * Installing to $TOOLS_PATH"
# limit make to number of cpus
MAKE='make -j '$(($(cat /proc/cpuinfo | grep processor | wc -l) - 2))

## paths for the tools
TOOLS_BIN_PATH="$TOOLS_PATH/bin"
TOOLS_INC_PATH="$TOOLS_PATH/include"
TOOLS_LIB_PATH="$TOOLS_PATH/lib"
TOOLS_SRC_PATH="$TOOLS_PATH/src"
TOOLS_LOG_PATH="$TOOLS_PATH/logs"
TOOLS_PATCHED_PATH=$(dirname $0)"/patched_files"

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

PCL_PATH="$TOOLS_SRC_PATH/pcl"
ODM_MESHING_PATH="$TOOLS_SRC_PATH/odm_meshing"
ODM_TEXTURING_PATH="$TOOLS_SRC_PATH/odm_texturing"
ODM_ORTHOPHOTO_PATH="$TOOLS_SRC_PATH/odm_orthophoto"
ODM_EXTRACT_UTM_PATH="$TOOLS_SRC_PATH/odm_extract_utm"
ODM_GEOREF_PATH="$TOOLS_SRC_PATH/odm_georef"

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

if [ "$CLEAN_BUILD_DIR" -ne 0 ] 
then 
    ## removing old stuff
    rm -Rf "$TOOLS_BIN_PATH"
    rm -Rf "$TOOLS_INC_PATH"
    rm -Rf "$TOOLS_LIB_PATH"
    rm -Rf "$TOOLS_SRC_PATH"
    rm -Rf "$TOOLS_LOG_PATH"
fi;

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

if [ "$INSTALL_PACKAGES" -ne 0 ]
then 
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
      > "$TOOLS_LOG_PATH/apt-get_install.log" 2>&1
    fi

    echo "  < done - `date`"
fi

## downloading sources
echo
echo "  > getting the sources"

## Reconstruct CMVS tar.gz from pieces...
#cat cmvs.tar.gz.part-?? > cmvs.tar.gz

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
bundler.zip https://github.com/snavely/bundler_sfm/archive/master.zip
PoissonRecon.zip http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version2/PoissonRecon.zip
vlfeat.tar.gz http://www.vlfeat.org/download/vlfeat-0.9.20-bin.tar.gz
cmvs.tar.gz http://www.di.ens.fr/cmvs/cmvs-fix2.tar.gz
graclus.tar.gz http://smathermather.github.io/BundlerTools/patched_files/src/graclus/graclus1.2.tar.gz
EOF

#bundler.zip  http://phototour.cs.washington.edu/bundler/distr/bundler-v0.4-source.zip
#parallel.tar.bz2  http://ftp.gnu.org/gnu/parallel/parallel-20141022.tar.bz2
#clapack.tgz  http://www.netlib.org/clapack/clapack-3.2.1-CMAKE.tgz
#pcl.tar.gz https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz


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
#mv -f clapack-3.2.1-CMAKE "$CLAPACK_PATH"
mv -f vlfeat-0.9.20       "$VLFEAT_PATH"
mv -f bundler-v0.4-source "$BUNDLER_PATH"
#mv -f parallel-20141022   "$PARALLEL_PATH"
mv -f PoissonRecon        "$PSR_PATH"
mv -f cmvs                "$CMVS_PATH"
#mv -f pcl-pcl-1.7.2       "$PCL_PATH"


echo "  < done - `date`"


## copying patches
echo
echo "  - copying patches"
echo

for file in `find $TOOLS_PATCHED_PATH -type f -print` ; do
  cp -v $file $TOOLS_PATH/${file/$TOOLS_PATCHED_PATH/.}
done

echo "  < done - `date`"


# building
echo
echo "  - building"
echo

chown -R `id -u`:`id -g` *
#sudo chmod -R 777 *

echo "  > graclus"
	cd "$GRACLUS_PATH"

	if [ "$ARCH" = "i686" ]; then
		sed -i "Makefile.in" -e "11c\COPTIONS = -DNUMBITS=32"
	fi

	if [ "$ARCH" = "x86_64" ]; then
		sed -i "Makefile.in" -e "11c\COPTIONS = -DNUMBITS=64"
	fi
	
	echo "    - cleaning graclus"
	${MAKE} clean > "graclus_1_clean.log" 2>&1

	echo "    - building graclus"
	${MAKE}  > "graclus_2_build.log" 2>&1
   
	mkdir "$TOOLS_INC_PATH/metisLib"
	cp -f "$GRACLUS_PATH/metisLib/"*.h "$TOOLS_INC_PATH/metisLib/"

	cp -f lib* "$TOOLS_LIB_PATH/"
echo "  < done - `date`"
echo

echo "  > poisson surface reconstruction "
  cd "$PSR_PATH"
  
  sed -i "$PSR_PATH/Makefile" -e "21c\BIN = ./"
    
  echo "    - building poisson surface reconstruction"
  ${MAKE} > "$TOOLS_LOG_PATH/poisson_1_build.log" 2>&1
  
  cp -f "$PSR_PATH/PoissonRecon" "$TOOLS_BIN_PATH/PoissonRecon"
  
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
  ${MAKE} clean > "$TOOLS_LOG_PATH/cmvs_1_clean.log" 2>&1

  echo "    - building cmvs"
  (${MAKE} || die "Error during building cmvs/pmvs") | tee "$TOOLS_LOG_PATH/cmvs_2_build.log"

  echo "    - make depend cmvs"
  ${MAKE} depend > "$TOOLS_LOG_PATH/cmvs_3_depend.log" 2>&1

  cp -f "$CMVS_PATH/program/main/cmvs" "$CMVS_PATH/program/main/pmvs2" "$CMVS_PATH/program/main/genOption" "$TOOLS_BIN_PATH/"
  cp -f "$CMVS_PATH/program/main/"*so* "$TOOLS_LIB_PATH/"
echo "  < done - `date`"
echo


echo "  > bundler"
  cd "$BUNDLER_PATH"

  sed -i "$BUNDLER_PATH/src/BundlerApp.h" -e "620c\        BundlerApp();"

  echo "    - cleaning bundler"
  ${MAKE} clean > "$TOOLS_LOG_PATH/bundler_1_clean.log" 2>&1

  echo "    - building bundler"
  (${MAKE} || die "Error ") | tee "$TOOLS_LOG_PATH/bundler_2_build.log"

  cp -f "$BUNDLER_PATH/bin/Bundle2PMVS" "$BUNDLER_PATH/bin/Bundle2Vis" "$BUNDLER_PATH/bin/KeyMatchFull" "$BUNDLER_PATH/bin/KeyMatch" "$BUNDLER_PATH/bin/bundler" "$BUNDLER_PATH/bin/RadialUndistort" "$TOOLS_BIN_PATH/"

  cp -f "$BUNDLER_PATH/lib/libANN_char.so" "$TOOLS_LIB_PATH/"
echo "  < done - `date`"
echo


echo "  > meshing "
	cd "$ODM_MESHING_PATH"
	
	echo "    - configuring odm_meshing"
    pkg-config pcl_common-1.7 --exists|| die "Please install PCL package"

    PCL_DIR=$(pkg-config pcl_common-1.7 --cflags-only-I | sed -e 's/\-I//' )

	(cmake . -DPCL_DIR="$PCL_DIR" || die "Error during meshing config") | tee "$TOOLS_LOG_PATH/odm_meshing_configure.log"
	
	echo "    - building odm_meshing"
	(${MAKE} || die "Error when compiling odm_meshing") | tee  "$TOOLS_LOG_PATH/odm_meshing_2_build.log"
	
	# copy output program to the binaries folder.
	cp -f "odm_meshing" "$TOOLS_BIN_PATH/odm_meshing"	
	
echo "  < done - `date`"
echo

echo "  > texturing "
	cd "$ODM_TEXTURING_PATH"
	
	echo "    - configuring odm_texturing"
	cmake . -DPCL_DIR="$TOOLS_LIB_PATH/pcl" > "$TOOLS_LOG_PATH/odm_texturing_1_build.log" 2>&1
	
	echo "    - building odm_texturing"
	${MAKE} > "$TOOLS_LOG_PATH/odm_texturing_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_texturing" "$TOOLS_BIN_PATH/odm_texturing"	
	
echo "  < done - `date`"
echo

echo "  > extract_utm "
	cd "$ODM_EXTRACT_UTM_PATH"
	
	echo "    - configuring odm_extract_utm"
	cmake . > "$TOOLS_LOG_PATH/odm_extract_utm_1_build.log" 2>&1
	
	echo "    - building odm_extract_utm"
	${MAKE} > "$TOOLS_LOG_PATH/odm_extract_utm_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_extract_utm" "$TOOLS_BIN_PATH/odm_extract_utm"	
	
echo "  < done - `date`"
echo

echo "  > georef "
	cd "$ODM_GEOREF_PATH"
	
	echo "    - configuring odm_georef"
	cmake . -DPCL_DIR="$TOOLS_LIB_PATH/pcl" > "$TOOLS_LOG_PATH/odm_georef_1_build.log" 2>&1
	
	echo "    - building odm_georef"
	${MAKE} > "$TOOLS_LOG_PATH/odm_georef_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_georef" "$TOOLS_BIN_PATH/odm_georef"	
	
echo "  < done - `date`"
echo

echo "  > orthophoto "
	cd "$ODM_ORTHOPHOTO_PATH"
	
	echo "    - configuring odm_orthophoto"
	cmake . -DPCL_DIR="$TOOLS_LIB_PATH/pcl" > "$TOOLS_LOG_PATH/odm_orthophoto_1_build.log" 2>&1
	
	echo "    - building odm_orthophoto"
	${MAKE} > "$TOOLS_LOG_PATH/odm_orthophoto_2_build.log" 2>&1
	
	# copy output program to the binaries folder.
	cp -f "odm_orthophoto" "$TOOLS_BIN_PATH/odm_orthophoto"	
	
echo "  < done - `date`"
echo

pushd "$TOOLS_PATH"

do_install 'bin/*' bin
do_install 'lib/*' lib
do_install ../run.pl bin

#nstall -o $(id -u) -g $(id -g) -m 755 -C -t "$DEST_PATH/$src" 
#ldconfig -v | tee "$TOOLS_LOG_PATH/ldconfig.log"
#chown -R $(id -u):$(id -g) *
#sudo chmod -R 777 *
popd

#chmod 755 run.pl

echo "  - script finished - `date`"
