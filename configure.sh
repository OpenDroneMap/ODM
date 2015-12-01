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
                     pkg-config -y
if [ $? -ne 0 ] 
then
    echo -e "\e[1;31mERROR: \e[39mWhen Installing Required Requisites\e[0m"
    exit 1
fi

## Installing Optional Requisites
echo -e "\e[1;34mInstalling OpenCV Dependencies\e[0;39m"
sudo apt-get install libgtk2.0-dev \
                     libavcodec-dev \
                     libavformat-dev \
                     libswscale-dev \
					 python-dev \
                     python-numpy \
                     libtbb2 \
                     libtbb-dev \
                     libjpeg-dev \
                     libpng-dev \
                     libtiff-dev \
                     libjasper-dev \
                     libflann-dev \
                     libproj-dev \
					 libxext-dev \
                     liblapack-dev \
                     libeigen3-dev \
                     libvtk5-dev -y
if [ $? -ne 0 ] 
then
    echo -e "\e[1;31mERROR: \e[39mError when Installing Dependencies Requisites\e[0m"
    exit 1
fi

## Remove libdc1394-22-dev due to python opencv issue
echo -e "\e[1;34mRemoving libdc1394-22-dev\e[0;39m"
sudo apt-get remove libdc1394-22-dev

## Installing OpenSfM Requisites
echo -e "\e[1;34mInstalling OpenSfM Dependencies\e[0;39m"
sudo apt-get install python-networkx \
                     libgoogle-glog-dev \
                     libsuitesparse-dev \
                     libboost-all-dev \
					 libboost-python-dev -y

sudo pip install -U PyYAML \
                    exifread \
                    gpxpy \
                    xmltodict
if [ $? -ne 0 ] 
then
    echo -e "\e[1;31mERROR: \e[39mError when Installing OpenSfM Dependencies\e[0m"
    exit 1
fi

## Installing Ecto Requisites
echo -e "\e[1;34mInstalling Ecto Dependencies\e[0;39m"
sudo pip install -U catkin-pkg
sudo apt-get install python-empy -y
if [ $? -ne 0 ] 
then
    echo -e "\e[1;31mERROR: \e[39mError when Installing Ecto Dependencies\e[0m"
    exit 1
fi

## Installing OpenDroneMap Requisites
echo -e "\e[1;34mInstalling OpenDroneMap Dependencies\e[0;39m"
sudo apt-get install python-pyexiv2 \
                     python-scipy -y
if [ $? -ne 0 ] 
then
    echo -e "\e[1;31mERROR: \e[39mError when Installing OpenDroneMap Dependencies\e[0m"
    exit 1
fi

## Get sys vars
NUM_CORES=`grep -c processor /proc/cpuinfo`

## Compile SuperBuild
cd SuperBuild 
mkdir -p build && cd build
cmake .. && make -j ${NUM_CORES}