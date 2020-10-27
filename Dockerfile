FROM ubuntu:18.04

# Env variables
ENV DEBIAN_FRONTEND noninteractive
ENV PYTHONPATH "$PYTHONPATH:/code/SuperBuild/install/lib/python3.6/dist-packages"
ENV PYTHONPATH "$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH "$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

# Prepare directories
WORKDIR /code

# Copy everything
COPY . ./

RUN bash configure.sh install

# Cleanup APT
RUN apt-get clean \
  && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Clean Superbuild
RUN rm -rf \
  /code/SuperBuild/build/opencv \
  /code/SuperBuild/download \
  /code/SuperBuild/src/ceres \
  /code/SuperBuild/src/mvstexturing \
  /code/SuperBuild/src/opencv \
  /code/SuperBuild/src/opengv \
  /code/SuperBuild/src/pcl \
  /code/SuperBuild/src/pdal \
  /code/SuperBuild/src/openmvs \
  /code/SuperBuild/build/openmvs

# Entry point
ENTRYPOINT ["python3", "/code/run.py"]