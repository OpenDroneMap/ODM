#!/bin/bash

check_version(){
  UBUNTU_VERSION=$(lsb_release -r)
  if [[ $UBUNTU_VERSION = *"18.04"* ]]; then
    echo "Ubuntu: $UBUNTU_VERSION, good!"
  elif [[ $UBUNTU_VERSION = *"16.04" ]]; then
    echo "ODM 2.0 has upgraded to Ubuntu 18.04, but you're on 16.04"
    echo "The last version of ODM that supports Ubuntu 16.04 is v1.0.2. We recommend you upgrade to Ubuntu 18.04, or better yet, use docker."
    exit 1
  else
    echo "You are not on Ubuntu 18.04 (detected: $UBUNTU_VERSION)"
    echo "It might be possible to run ODM on a newer version of Ubuntu, however, you cannot rely on this script."
    exit 1
  fi
}

if [[ $2 =~ ^[0-9]+$ ]] ; then
    processes=$2
else
    processes=$(nproc)
fi

install() {
    cd /code
    
    ## Set up library paths
    export DEBIAN_FRONTEND=noninteractive
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RUNPATH/SuperBuild/install/lib

	## Before installing
    echo "Updating the system"
    if ! command -v sudo &> /dev/null
    then
        echo "Installing sudo"
        apt-get update && apt-get install -y sudo
    fi
    sudo apt-get update && sudo apt-get install software-properties-common lsb-release tzdata -y  --no-install-recommends
    
    # Check version
    check_version
    
    sudo add-apt-repository -y ppa:ubuntugis/ubuntugis-unstable
    sudo apt-get update
    
    echo "Installing Required Requisites"
    sudo apt-get install -y -qq --no-install-recommends \
                         build-essential \
                         git \
                         cmake \
                         python3-pip \
                         libgdal-dev \
                         gdal-bin \
                         libgeotiff-dev \
                         pkg-config \
                         libjsoncpp-dev \
                         python3-gdal \
                         python3-setuptools \
                         grass-core \
                         libssl-dev \
                         swig3.0 \
                         python3-wheel \
                         libboost-log-dev
    sudo pip3 install -U pip


    echo "Installing OpenCV Dependencies"
    sudo apt-get install -y -qq --no-install-recommends libgtk2.0-dev \
                         libavcodec-dev \
                         libavformat-dev \
                         libswscale-dev \
                         python3-dev \
                         libtbb2 \
                         libtbb-dev \
                         libjpeg-dev \
                         libpng-dev \
                         libtiff-dev \
                         libflann-dev \
                         libproj-dev \
                         libxext-dev \
                         liblapack-dev \
                         libeigen3-dev \
                         libvtk6-dev
	
    echo "Installing OpenSfM Dependencies"
    sudo apt-get install -y -qq  --no-install-recommends libgoogle-glog-dev \
                         libsuitesparse-dev \
                         libboost-filesystem-dev \
                         libboost-iostreams-dev \
                         libboost-regex-dev \
                         libboost-python-dev \
                         libboost-date-time-dev \
                         libboost-thread-dev

    pip install --ignore-installed -r requirements.txt

    if [ ! -z "$PORTABLE_INSTALL" ]; then
        echo "Replacing g++ and gcc with our scripts for portability..."
        if [ ! -e /usr/bin/gcc_real ]; then
            sudo mv -v /usr/bin/gcc /usr/bin/gcc_real
            sudo cp -v ./docker/gcc /usr/bin/gcc
        fi
        if [ ! -e /usr/bin/g++_real ]; then
            sudo mv -v /usr/bin/g++ /usr/bin/g++_real
            sudo cp -v ./docker/g++ /usr/bin/g++
        fi
    fi

    set -eo pipefail
    
    echo "Compiling SuperBuild"
    cd ${RUNPATH}/SuperBuild
    mkdir -p build && cd build
    cmake .. && make -j$processes

    echo "Compiling build"
    cd ${RUNPATH}
    mkdir -p build && cd build
    cmake .. && make -j$processes
	
    echo "Configuration Finished"
}

uninstall() {
    check_version

    echo "Removing SuperBuild and build directories"
    cd ${RUNPATH}/SuperBuild
    rm -rfv build src download install
    cd ../
    rm -rfv build
}

reinstall() {
    check_version

    echo "Reinstalling ODM modules"
    uninstall
    install
}
usage() {
    echo "Usage:"
    echo "bash configure.sh <install|update|uninstall|help> [nproc]"
    echo "Subcommands:"
    echo "  install"
    echo "    Installs all dependencies and modules for running OpenDroneMap"
    echo "  reinstall"
    echo "    Removes SuperBuild and build modules, then re-installs them. Note this does not update OpenDroneMap to the latest version. "
    echo "  uninstall"
    echo "    Removes SuperBuild and build modules. Does not uninstall dependencies"
    echo "  help"
    echo "    Displays this message"
    echo "[nproc] is an optional argument that can set the number of processes for the make -j tag. By default it uses $(nproc)"
}

if [[ $1 =~ ^(install|reinstall|uninstall)$ ]]; then
    RUNPATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    "$1"
else
    echo "Invalid instructions." >&2
    usage
    exit 1
fi
