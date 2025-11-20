FROM ubuntu:24.04 AS builder

# Env variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python3.12/dist-packages:/code/SuperBuild/install/bin/opensfm" \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"
    CC="ccache gcc" \
    CXX="ccache g++" \
    CCACHE_DIR=/ccache

# Prepare directories
WORKDIR /code

# Install ccache first for all subsequent builds
RUN apt-get update && \
    apt-get install -y --no-install-recommends ccache && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy only what's needed for dep installation
COPY snap/ ./snap/
COPY configure.sh ./
COPY requirements.txt ./

# Install system deps and Python requirements
# Layer is cached unless these files change
RUN bash configure.sh installreqs

# Copy SuperBuild config
COPY SuperBuild/ ./SuperBuild/

# Build SuperBuild (heavy compilation step),
# with cache for faster rebuilds
RUN --mount=type=cache,target=/ccache \
    cd SuperBuild && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc)

# Copy app code late, to avoid cache busting
COPY . ./

# Run the tests
ENV PATH="/code/venv/bin:$PATH"
RUN bash test.sh

# Clean SuperBuild temp files
RUN bash configure.sh clean

### END Builder

### Use a second image for the final asset to reduce the number and
# size of the layers.
FROM ubuntu:24.04

# Env variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python3.12/dist-packages:/code/SuperBuild/install/bin/opensfm" \
    LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib" \
    PDAL_DRIVER_PATH="/code/SuperBuild/install/bin"

WORKDIR /code

# Copy everything we built from the builder
COPY --from=builder /code /code

ENV PATH="/code/venv/bin:$PATH"

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
