FROM ubuntu:14.04
MAINTAINER Danilo Bargen <mail@dbrgn.ch>

# Env variables
ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get update \
    && sudo apt-get remove libdc1394-22-dev \
    && apt-get install -y --install-recommends \
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
RUN bash ./configure.sh && \
    chown -R odm:odm /code
USER odm

ENV PYTHONPATH=${PYTHONPATH}:/code/SuperBuild/install/lib/python2.7/dist-packages:/code/SuperBuild/src/opensfm \
    LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/code/SuperBuild/install/lib

# Entry point
VOLUME ["/images"]
# WORKDIR /images
ENTRYPOINT ["python", "/code/run.py", "--project-path", "/images"]