#!/bin/bash

## Before installing
echo "Updating the system"
sudo apt-get update

echo "Installing Required Requisites"
sudo apt-get install -y build-essential \
                     cmake \
                     git \
                     python-pip \
                     libgdal-dev \
                     gdal-bin \
                     libgeotiff-dev \
                     pkg-config -y -qq

echo "Upgrading CMake for MVS-Texturing"
sudo apt-get install -y software-properties-common python-software-properties
sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
sudo apt-get update -y
sudo apt-get install cmake -y

echo "Installing OpenCV Dependencies"
sudo apt-get install -y libgtk2.0-dev \
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
                     libvtk5-dev -y -qq

echo "Removing libdc1394-22-dev due to python opencv issue"
sudo apt-get remove libdc1394-22-dev

## Installing OpenSfM Requisites
echo "Installing OpenSfM Dependencies"
sudo apt-get install -y python-networkx \
                     libgoogle-glog-dev \
                     libsuitesparse-dev \
                     libboost-filesystem-dev \
                     libboost-iostreams-dev \
                     libboost-regex-dev \
                     libboost-python-dev \
                     libboost-date-time-dev \
                     libboost-thread-dev -y -qq

sudo pip install -U PyYAML \
                    exifread \
                    gpxpy \
                    xmltodict

echo "Installing Ecto Dependencies"
sudo pip install -U catkin-pkg
sudo apt-get install -y python-empy \
                     python-nose \
                     python-pyside -y -qq

echo "Installing OpenDroneMap Dependencies"
sudo apt-get install -y python-pyexiv2 \
                     python-scipy \
                     jhead \
                     liblas-bin -y -qq

echo "Compiling SuperBuild"
cd SuperBuild
mkdir -p build && cd build
cmake .. && make -j8

echo "Configuration Finished"
