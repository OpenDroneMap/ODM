#!/bin/bash

# Check OS
if [ ! $(command -v apt-get) ]; then
  echo -e "\e[1;31mERROR: Not a Debian-based linux system. 
           Impossible to install OpenCV with this script\e[0;39m"
  exit 1
fi

## Before installing
echo -e "\e[1;34mUpdating the system\e[0;39m"
sudo apt-get update
END_CMD1=$?
sudo apt-get upgrade -y
END_CMD2=$?
if [ $END_CMD1 -ne 0 -o $END_CMD2 -ne 0 ]
then
	echo -e "\e[1;31mERROR: \e[39mWhen Updating the system\e[0m"
	exit 1
fi

## Install Required Requisites
echo -e "\e[1;34mInstalling Required Requisites\e[0;39m"
sudo apt-get install build-essential \
                     cmake \
                     git \
                     python-pip \
                     pkg-config \
                     libavcodec-dev \
                     libavformat-dev \
                     libswscale-dev -y
if [ $? -ne 0 ] 
then
    echo -e "\e[1;31mERROR: \e[39mWhen Installing Required Requisites\e[0m"
    exit 1
fi

## Installing Optional Requisites
echo -e "\e[1;34mInstalling Optional Requisites\e[0;39m"
sudo apt-get install python-dev \
                     python-numpy \
                     libtbb2 \
                     libtbb-dev \
                     libjpeg-dev \
                     libpng-dev \
                     libtiff-dev \
                     libjasper-dev \
                     libflann-dev \
                     libdc1394-22-dev \
                     libboost-all-dev \
					 libboost-python-dev \
					 libxext-dev \
                     liblapack-dev \
                     libeigen3-dev \
                     libgtk2.0-dev \
                     libvtk5-dev \
                     libgoogle-glog-dev \
                     libsuitesparse-dev -y
if [ $? -ne 0 ] 
then
    echo -e "\e[1;33mWARNING: \e[39mError when Installing Optional Requisites\e[0m"
fi

## Get sys vars
NUM_CORES=`grep -c processor /proc/cpuinfo`

## Compile SuperBuild
cd SuperBuild 
mkdir -p build && cd build
cmake .. && make -j ${NUM_CORES}