FROM ubuntu:14.04
MAINTAINER Danilo Bargen <mail@dbrgn.ch>

# Env variables
ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get update \
    && apt-get install -y --install-recommends \
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
        libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev \
    && apt-get autoremove \
    && apt-get clean

# Add users
RUN useradd -m -U odm

# Prepare directories
RUN mkdir /code
WORKDIR /code

# Add repository files
ADD . /code/

# Update submodules
RUN git submodule init && git submodule update

# Build OpenDroneMap
RUN ./install.sh && \
    chown -R odm:odm /code
USER odm

# Entry point
VOLUME ["/images"]
WORKDIR /images
ENTRYPOINT ["/code/run.py"]
