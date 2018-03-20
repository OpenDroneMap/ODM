FROM phusion/baseimage

# Env variables
ENV DEBIAN_FRONTEND noninteractive

#Install dependencies
#Required Requisites
RUN add-apt-repository -y ppa:ubuntugis/ppa
RUN add-apt-repository -y ppa:george-edison55/cmake-3.x
RUN apt-get update -y

# All packages (Will install much faster)
RUN apt-get install --no-install-recommends -y git cmake python-pip build-essential software-properties-common python-software-properties libgdal-dev gdal-bin libgeotiff-dev \
libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libflann-dev \
libproj-dev libxext-dev liblapack-dev libeigen3-dev libvtk6-dev python-networkx libgoogle-glog-dev libsuitesparse-dev libboost-filesystem-dev libboost-iostreams-dev \
libboost-regex-dev libboost-python-dev libboost-date-time-dev libboost-thread-dev python-pyproj python-empy python-nose python-pyside python-pyexiv2 python-scipy \
libexiv2-dev liblas-bin python-matplotlib libatlas-base-dev swig2.0 python-wheel libboost-log-dev libjsoncpp-dev python-gdal

RUN apt-get remove libdc1394-22-dev
RUN pip install --upgrade pip
RUN pip install setuptools
RUN pip install -U PyYAML exifread gpxpy xmltodict catkin-pkg appsettings https://github.com/OpenDroneMap/gippy/archive/v0.3.9.tar.gz loky scipy shapely numpy pyproj psutil

ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

# Prepare directories

RUN mkdir /code
WORKDIR /code

# Copy repository files
COPY ccd_defs_check.py /code/ccd_defs_check.py
COPY CMakeLists.txt /code/CMakeLists.txt
COPY configure.sh /code/configure.sh
COPY /modules/ /code/modules/
COPY /opendm/ /code/opendm/
COPY /patched_files/ /code/patched_files/
COPY run.py /code/run.py
COPY run.sh /code/run.sh
COPY /scripts/ /code/scripts/
COPY /SuperBuild/cmake/ /code/SuperBuild/cmake/
COPY /SuperBuild/CMakeLists.txt /code/SuperBuild/CMakeLists.txt
COPY docker.settings.yaml /code/settings.yaml
COPY VERSION /code/VERSION

# Replace g++ and gcc with our own scripts
COPY /docker/ /code/docker/
RUN mv -v /usr/bin/gcc /usr/bin/gcc_real && mv -v /usr/bin/g++ /usr/bin/g++_real && cp -v /code/docker/gcc /usr/bin/gcc && cp -v /code/docker/g++ /usr/bin/g++

#Compile code in SuperBuild and root directories

RUN cd SuperBuild && mkdir build && cd build && cmake  .. && make -j$(nproc)     && cd ../.. && mkdir build && cd build && cmake .. && make -j$(nproc)

RUN apt-get -y remove libgl1-mesa-dri git cmake python-pip build-essential
RUN apt-get install -y libvtk6-dev

# Cleanup APT
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Clean Superbuild

RUN rm -rf /code/SuperBuild/download /code/SuperBuild/src/vtk7 /code/SuperBuild/src/opencv /code/SuperBuild/src/pcl /code/SuperBuild/src/pdal /code/SuperBuild/src/opengv /code/SuperBuild/src/mvstexturing /code/SuperBuild/src/ceres /code/SuperBuild/build/vtk7 /code/SuperBuild/build/opencv

# Entry point
ENTRYPOINT ["python", "/code/run.py", "code"]

