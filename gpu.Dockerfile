# GPU image: pixi build with cuda-toolkit, NVIDIA runtime for execution.

FROM ghcr.io/prefix-dev/pixi:latest AS dev

RUN if id "ubuntu" >/dev/null 2>&1; then \
        userdel -f -r ubuntu || echo "Failed to delete ubuntu user"; \
    fi

WORKDIR /code

# No NVIDIA driver is present during the build, so pixi cannot detect the __cuda
# virtual package the gpu environments require. The driver is supplied at runtime
# by the NVIDIA container runtime; mock it here to match the cuda floor in pixi.toml.
ENV CONDA_OVERRIDE_CUDA=12.0

FROM dev AS builder

COPY pixi.toml pixi.lock ./
RUN pixi install --locked -e gpu \
    && pixi install --locked -e gpu-prod \
    && mkdir -p scripts \
    && pixi shell-hook -e gpu-prod -s bash > scripts/pixi-shell-hook \
    && rm -rf .pixi/envs/gpu-prod/include .pixi/envs/gpu-prod/share/doc .pixi/envs/gpu-prod/share/man .pixi/envs/gpu-prod/share/info

COPY . ./
ARG CMAKE_BUILD_PARALLEL_LEVEL
RUN pixi run -e gpu build && pixi run -e gpu test

RUN mkdir -p /odm-runtime/SuperBuild /odm-runtime/scripts \
    && cp -a SuperBuild/install /odm-runtime/SuperBuild/ \
    && cp -a opendm stages contrib /odm-runtime/ \
    && cp run.py settings.yaml VERSION /odm-runtime/ \
    && cp scripts/docker-entrypoint.sh scripts/smoke.py /odm-runtime/scripts/

FROM nvidia/cuda:12.9.1-runtime-ubuntu24.04 AS runtime

# revision, version and created are stamped at build time: by metadata-action
# in the publish workflows, by scripts/docker-build.py locally.
LABEL org.opencontainers.image.source="https://github.com/OpenDroneMap/ODM" \
      org.opencontainers.image.title="OpenDroneMap" \
      org.opencontainers.image.description="Command line toolkit for processing aerial drone imagery" \
      org.opencontainers.image.url="https://www.opendronemap.org" \
      org.opencontainers.image.documentation="https://docs.opendronemap.org" \
      org.opencontainers.image.licenses="AGPL-3.0-only" \
      org.opencontainers.image.vendor="OpenDroneMap"

ENV DEBIAN_FRONTEND=noninteractive
ENV PIXI_ENV=gpu-prod

WORKDIR /code

COPY --from=builder /code/.pixi/envs/gpu-prod .pixi/envs/gpu-prod
COPY --from=builder /code/scripts/pixi-shell-hook scripts/pixi-shell-hook
COPY --from=builder /odm-runtime/ ./

# The smoke test launches the CUDA-linked OpenMVS binaries, which need
# libcuda.so.1. No driver is injected during a build, so fall back to the compat
# driver this base image ships but leaves off the loader path. Keep it scoped to
# this step: at runtime the host driver must take precedence over compat.
RUN chmod +x scripts/docker-entrypoint.sh run.py \
    && LD_LIBRARY_PATH=/usr/local/cuda/compat \
       bash scripts/docker-entrypoint.sh python3 scripts/smoke.py

ENTRYPOINT ["/code/scripts/docker-entrypoint.sh"]
