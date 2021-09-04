FROM ubuntu:21.04 AS builder

# Env variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python3.9/dist-packages:/code/SuperBuild/install/lib/python3.8/dist-packages:/code/SuperBuild/src/opensfm" \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

# Prepare directories
WORKDIR /code

# Copy everything
COPY . ./

# Run the build
RUN bash configure.sh install

# Clean Superbuild
RUN bash configure.sh clean

### END Builder

### Use a second image for the final asset to reduce the number and
# size of the layers.
FROM ubuntu:21.04

# Env variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python3.9:/code/SuperBuild/install/lib/python3.8/dist-packages:/code/SuperBuild/src/opensfm" \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

WORKDIR /code

# Copy everything we built from the builder
COPY --from=builder /code /code

# Copy the Python libraries installed via pip from the builder
COPY --from=builder /usr/local /usr/local

# Install shared libraries that we depend on via APT, but *not*
# the -dev packages to save space!
RUN bash configure.sh installruntimedepsonly \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Entry point
ENTRYPOINT ["python3", "/code/run.py"]
