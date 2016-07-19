#Help for users:
#BUILD COMMAND EXAMPLE: docker build -t alex1 .
#AUTOMATIC RUN COMMAND EXAMPLE: docker run -it -v /home/alex/OpenDroneMap/images:/code/images --rm alex1
#MANUAL RUN COMMAND EXAMPLE: docker run -it -v /home/alex/OpenDroneMap/images:/code/images --rm --entrypoint bash alex1

FROM ubuntu:14.04
MAINTAINER Danilo Bargen <mail@dbrgn.ch>
MAINTAINER Alex Hagiopol <alex.hagiopol@icloud.com>

# Env variables
ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get update \
    && sudo apt-get remove libdc1394-22-dev \
    && apt-get install -y \
        build-essential \
                     cmake \
                     git \
                     python-pip \
                     libgdal-dev \
                     libgeotiff-dev \
                     pkg-config \
                     libgtk2.0-dev \
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
                     libvtk5-dev \
                     python-networkx \
                     libgoogle-glog-dev \
                     libsuitesparse-dev \
                     libboost-filesystem-dev \
                     libboost-iostreams-dev \
                     libboost-regex-dev \
                     libboost-python-dev \
                     libboost-date-time-dev \
                     libboost-thread-dev \
                     python-empy \
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
