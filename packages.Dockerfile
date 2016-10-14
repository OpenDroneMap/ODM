FROM ubuntu:14.04
MAINTAINER Alex Hagiopol <alex.hagiopol@icloud.com>

# Env variables
ENV DEBIAN_FRONTEND noninteractive

#Install dependencies
#Required Requisites
RUN apt-get update \
    && apt-get install -y -qq \
       build-essential \
       cmake \
       git \
       python-pip \
       libgdal-dev \
       gdal-bin \
       libgeotiff-dev \
       pkg-config

#CMake 3.1 for MVS-Texturing
RUN apt-get install -y software-properties-common python-software-properties
RUN add-apt-repository -y ppa:george-edison55/cmake-3.x
RUN apt-get update -y
RUN apt-get install -y --only-upgrade cmake

#Installing OpenCV Dependencies
RUN apt-get install -y -qq libgtk2.0-dev \
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
                     libvtk5-dev

#Removing libdc1394-22-dev due to python opencv issue
RUN apt-get remove libdc1394-22-dev

#Installing OpenSfM Dependencies
RUN apt-get install -y -qq python-networkx \
                     libgoogle-glog-dev \
                     libsuitesparse-dev \
                     libboost-filesystem-dev \
                     libboost-iostreams-dev \
                     libboost-regex-dev \
                     libboost-python-dev \
                     libboost-date-time-dev \
                     libboost-thread-dev \
                     python-pyproj
RUN pip install -U PyYAML \
                    exifread \
                    gpxpy \
                    xmltodict \
                    catkin-pkg

#Installing Ecto Dependencies
RUN apt-get install -y -qq python-empy \
                     python-nose \
                     python-pyside

#"Installing OpenDroneMap Dependencies"
RUN apt-get install -y python-pyexiv2 \
                     python-scipy \
                     jhead \
                     liblas-bin -y -qq

RUN apt-get install -y python-empy \
                     python-nose \
                     python-pyside \
                     python-pyexiv2 \
                     python-scipy \
                     jhead \
                     liblas-bin \
                     python-matplotlib \
                     libatlas-base-dev \
                     libatlas3gf-base

ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

