ARG BASE_IMAGE=mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04
ARG RUNTIME_BASE_IMAGE=ubuntu:24.04

FROM ${BASE_IMAGE} AS dev

# Build toggles:
# - PORTABLE_INSTALL=YES enables the portable compiler wrappers.
ARG PORTABLE_INSTALL=NO

RUN if id "ubuntu" &>/dev/null; then \
        echo "Deleting user 'ubuntu'" && userdel -f -r ubuntu || echo "Failed to delete ubuntu user"; \
    else \
         echo "User 'ubuntu' does not exist"; \
    fi

WORKDIR /code

FROM dev AS builder

# Copy everything
COPY . ./

# Run the build
RUN PORTABLE_INSTALL=${PORTABLE_INSTALL} bash configure.sh install

# Run the tests
ENV PATH="/code/venv/bin:$PATH"
RUN bash test.sh

### END Builder

### Use a second image for the final asset to reduce the number and
# size of the layers.
FROM ${RUNTIME_BASE_IMAGE} AS runtime

# Optional runtime packages for image variants
ARG EXTRA_RUNTIME_PACKAGES=

# Env variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/local/lib/python3.12/dist-packages:/code/SuperBuild/install/lib/python3.12/dist-packages:/code/SuperBuild/install/bin/opensfm" \
    LD_LIBRARY_PATH="/code/SuperBuild/install/lib" \
    PDAL_DRIVER_PATH="/code/SuperBuild/install/bin"

WORKDIR /code

# Copy everything we built from the builder
COPY --from=builder /code /code

ENV PATH="/code/venv/bin:$PATH"

# Install shared libraries that we depend on via APT, but *not*
# the -dev packages to save space!
# Also run a smoke test on ODM and OpenSfM
RUN if [ -n "${EXTRA_RUNTIME_PACKAGES}" ]; then \
        apt-get update -y && apt-get install -y ${EXTRA_RUNTIME_PACKAGES}; \
    fi; \
    bash configure.sh installruntimedepsonly; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*; \
    bash run.sh --help; \
    bash -c "eval $(python3 /code/opendm/context.py) && python3 -c 'from opensfm import io, pymap'"

# Entry point
ENTRYPOINT ["python3", "/code/run.py"]
