FROM nvidia/cuda:12.9.1-devel-ubuntu24.04 AS builder

# Env variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/local/lib/python3.12/dist-packages:/code/SuperBuild/install/lib/python3.12/dist-packages:/code/SuperBuild/install/bin/opensfm" \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

# Prepare directories
WORKDIR /code

# Copy everything
COPY . ./

# Run the build
RUN PORTABLE_INSTALL=YES GPU_INSTALL=YES bash configure.sh install

# Run the tests
ENV PATH="/code/venv/bin:$PATH"
RUN bash test.sh

# Clean Superbuild
RUN bash configure.sh clean

### END Builder

### Use a second image for the final asset to reduce the number and
# size of the layers.
FROM nvidia/cuda:12.9.1-runtime-ubuntu24.04

ARG GIT_COMMIT=unknown
ARG BUILD_DATE=unknown
ARG IMAGE_VERSION=unknown
LABEL org.opencontainers.image.revision="$GIT_COMMIT" \
      org.opencontainers.image.source="https://github.com/OpenDroneMap/ODM" \
      org.opencontainers.image.version="$IMAGE_VERSION" \
      org.opencontainers.image.created="$BUILD_DATE" \
      org.opencontainers.image.title="OpenDroneMap" \
      org.opencontainers.image.description="Command line toolkit for processing aerial drone imagery" \
      org.opencontainers.image.url="https://www.opendronemap.org" \
      org.opencontainers.image.documentation="https://docs.opendronemap.org" \
      org.opencontainers.image.licenses="AGPL-3.0-only" \
      org.opencontainers.image.vendor="OpenDroneMap"

# Env variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/local/lib/python3.12/dist-packages:/code/SuperBuild/install/lib/python3.12/dist-packages:/code/SuperBuild/install/bin/opensfm" \
    # The following paths (/usr/local/nvidia/lib64 and /usr/local/nvidia/lib) 
    # are required for compatibility with Google Kubernetes Engine (GKE) managed drivers.
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib:/usr/local/nvidia/lib64:/usr/local/nvidia/lib" \
    PDAL_DRIVER_PATH="/code/SuperBuild/install/bin"

WORKDIR /code

# Copy everything we built from the builder
COPY --from=builder /code /code

# Include /usr/local/nvidia/bin in PATH for GKE compatibility
ENV PATH="/code/venv/bin:/usr/local/nvidia/bin:$PATH"

RUN apt-get update -y \
 && apt-get install -y ffmpeg libtbbmalloc2
# Install shared libraries that we depend on via APT, but *not*
# the -dev packages to save space!
# Also run a smoke test on ODM and OpenSfM
RUN bash configure.sh installruntimedepsonly \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
  && bash run.sh --help \
  && bash -c "eval $(python3 /code/opendm/context.py) && python3 -c 'from opensfm import io, pymap'"

# Entry point
ENTRYPOINT ["python3", "/code/run.py"]
