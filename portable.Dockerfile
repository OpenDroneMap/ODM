FROM phusion/baseimage

# Env variables
ENV DEBIAN_FRONTEND noninteractive

#Install dependencies and required requisites
RUN apt-get update -y \
  && apt-get install -y \
    software-properties-common \
  && add-apt-repository -y ppa:ubuntugis/ubuntugis-unstable \
  && add-apt-repository -y ppa:george-edison55/cmake-3.x \
  && apt-get update -y

# All packages (Will install much faster)
RUN apt-get install --no-install-recommends -y \
  build-essential \
  cmake \
  gdal-bin \
  git \
  libatlas-base-dev \
  libavcodec-dev \
  libavformat-dev \
  libboost-date-time-dev \
  libboost-filesystem-dev \
  libboost-iostreams-dev \
  libboost-log-dev \
  libboost-python-dev \
  libboost-regex-dev \
  libboost-thread-dev \
  libeigen3-dev \
  libflann-dev \
  libgdal-dev \
  libgeotiff-dev \
  libgoogle-glog-dev \
  libgtk2.0-dev \
  libjasper-dev \
  libjpeg-dev \
  libjsoncpp-dev \
  liblapack-dev \
  liblas-bin \
  libpng-dev \
  libproj-dev \
  libsuitesparse-dev \
  libswscale-dev \
  libtbb2 \
  libtbb-dev \
  libtiff-dev \
  libvtk6-dev \
  libxext-dev \
  python-dev \
  python-gdal \
  python-matplotlib \
  python-pip \
  python-software-properties \
  python-wheel \
  swig2.0 \
  grass-core \
  libssl-dev

RUN apt-get remove libdc1394-22-dev
RUN pip install --upgrade pip
RUN pip install setuptools

# Prepare directories
RUN mkdir /code
WORKDIR /code

# Copy repository files
COPY CMakeLists.txt /code/CMakeLists.txt
COPY configure.sh /code/configure.sh
COPY /modules/ /code/modules/
COPY /opendm/ /code/opendm/
COPY run.py /code/run.py
COPY run.sh /code/run.sh
COPY /stages/ /code/stages/
COPY /SuperBuild/cmake/ /code/SuperBuild/cmake/
COPY /SuperBuild/CMakeLists.txt /code/SuperBuild/CMakeLists.txt
COPY docker.settings.yaml /code/settings.yaml
COPY VERSION /code/VERSION
COPY requirements.txt /code/requirements.txt

RUN pip install -r requirements.txt
RUN pip install --upgrade cryptography && python -m easy_install --upgrade pyOpenSSL

ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

# Replace g++ and gcc with our own scripts
COPY /docker/ /code/docker/
RUN mv -v /usr/bin/gcc /usr/bin/gcc_real \
  && mv -v /usr/bin/g++ /usr/bin/g++_real \
  && cp -v /code/docker/gcc /usr/bin/gcc \
  && cp -v /code/docker/g++ /usr/bin/g++

# Compile code in SuperBuild and root directories
RUN cd SuperBuild \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j$(nproc) \
  && cd ../.. \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j$(nproc)

# Cleanup APT
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Clean Superbuild
RUN rm -rf \
  /code/SuperBuild/build/opencv \
  /code/SuperBuild/download \
  /code/SuperBuild/src/ceres \
  /code/SuperBuild/src/mvstexturing \
  /code/SuperBuild/src/opencv \
  /code/SuperBuild/src/opengv \
  /code/SuperBuild/src/pcl \
  /code/SuperBuild/src/pdal

# Entry point
ENTRYPOINT ["python", "/code/run.py"]

