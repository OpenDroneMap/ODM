# syntax=docker/dockerfile:1.4

FROM ubuntu:24.04 AS dev

RUN if id "ubuntu" &>/dev/null; then \
        echo "Deleting user 'ubuntu'" && userdel -f -r ubuntu || echo "Failed to delete ubuntu user"; \
    else \
         echo "User 'ubuntu' does not exist"; \
    fi

RUN apt-get update -y && apt-get install -y \
    curl \
    ca-certificates \
    && curl -fsSL https://pixi.sh/install.sh | env PIXI_HOME=/usr/local bash \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV PATH="/usr/local/bin:${PATH}"

WORKDIR /code

######## Builder ########
FROM dev AS builder

# SuperBuild is memory-heavy; cap parallel jobs (override with --build-arg).
ARG CMAKE_BUILD_PARALLEL_LEVEL=4
ENV CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL}

# Pixi/rattler package download cache (not the installed env — that stays in the layer).
ENV RATTLER_CACHE_DIR=/root/.cache/rattler/cache

# Layer cache: reinstall conda env only when the lockfile changes.
COPY pixi.toml pixi.lock ./
RUN --mount=type=cache,target=/root/.cache/rattler \
    pixi install --frozen

COPY . ./

# BuildKit cache mounts: reuse downloaded tarballs and extracted sources between builds.
# Do not cache SuperBuild/build — stamps there reference install/bin/opensfm (git checkout)
# which lives in the image layer; stale stamps skip the clone and break submodule update.
RUN --mount=type=cache,target=/code/SuperBuild/download \
    --mount=type=cache,target=/code/SuperBuild/src \
    --mount=type=cache,target=/root/.cache/rattler \
    pixi run build

RUN pixi run test

######## Runtime ########
FROM ubuntu:24.04 AS runtime

ARG HARDWARE

ENV DEBIAN_FRONTEND=noninteractive \
    CONDA_PREFIX=/code/.pixi/envs/default \
    PATH="/code/.pixi/envs/default/bin:/usr/local/bin:/usr/bin:/bin" \
    LD_LIBRARY_PATH="/code/.pixi/envs/default/lib:/code/SuperBuild/install/lib" \
    GDAL_DATA=/code/.pixi/envs/default/share/gdal \
    PROJ_LIB=/code/.pixi/envs/default/share/proj \
    PDAL_DRIVER_PATH=/code/SuperBuild/install/bin \
    PYTHONPATH="/code/SuperBuild/install/lib/python3.12/dist-packages:/code/SuperBuild/install/lib/python3/dist-packages:/code/SuperBuild/install/bin/opensfm"

WORKDIR /code

COPY --from=builder /code /code

RUN chmod +x /code/scripts/docker-entrypoint.sh /code/run.sh /code/scripts/smoke.sh \
    && bash /code/scripts/smoke.sh

ENTRYPOINT ["/code/scripts/docker-entrypoint.sh"]
