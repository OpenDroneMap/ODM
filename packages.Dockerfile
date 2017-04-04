FROM phusion/baseimage
MAINTAINER Alex Hagiopol <alex.hagiopol@icloud.com>

# Env variables
ENV DEBIAN_FRONTEND noninteractive

#Install dependencies
#Required Requisites
RUN add-apt-repository -y ppa:ubuntugis/ppa
RUN add-apt-repository -y ppa:george-edison55/cmake-3.x
RUN apt-get update -y

# All packages (Will install much faster)
RUN apt-get install -y build-essential cmake git python-pip pkg-config software-properties-common python-software-properties libgdal-dev gdal-bin libgeotiff-dev \
libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libflann-dev \
libproj-dev libxext-dev liblapack-dev libeigen3-dev libvtk5-dev python-networkx libgoogle-glog-dev libsuitesparse-dev libboost-filesystem-dev libboost-iostreams-dev \
libboost-regex-dev libboost-python-dev libboost-date-time-dev libboost-thread-dev python-pyproj python-empy python-nose python-pyside python-pyexiv2 python-scipy \
jhead liblas-bin python-matplotlib libatlas-base-dev

RUN apt-get remove libdc1394-22-dev

RUN pip install -U PyYAML \
                    exifread \
                    gpxpy \
                    xmltodict \
                    catkin-pkg \
                    appsettings


RUN apt-get clean -y


ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

