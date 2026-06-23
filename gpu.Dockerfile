# GPU image: pixi build with cuda-toolkit, NVIDIA runtime for execution.

FROM ghcr.io/prefix-dev/pixi:latest AS dev

RUN if id "ubuntu" >/dev/null 2>&1; then \
        userdel -f -r ubuntu || echo "Failed to delete ubuntu user"; \
    fi

WORKDIR /code

FROM dev AS builder

COPY pixi.toml pixi.lock ./
RUN pixi install --locked -e gpu

COPY . ./
RUN pixi run -e gpu build && pixi run -e gpu test

RUN mkdir -p /odm-runtime/SuperBuild /odm-runtime/scripts \
    && cp -a SuperBuild/install /odm-runtime/SuperBuild/ \
    && cp -a opendm stages /odm-runtime/ \
    && cp run.py run.sh settings.yaml VERSION /odm-runtime/ \
    && cp scripts/docker-entrypoint.sh scripts/smoke.py /odm-runtime/scripts/

FROM dev AS prod-env

COPY pixi.toml pixi.lock ./
RUN pixi install --locked -e gpu-prod \
    && mkdir -p scripts \
    && pixi shell-hook -e gpu-prod -s bash > scripts/pixi-shell-hook \
    && rm -rf .pixi/envs/gpu-prod/include .pixi/envs/gpu-prod/share/doc .pixi/envs/gpu-prod/share/man .pixi/envs/gpu-prod/share/info

FROM nvidia/cuda:12.9.1-runtime-ubuntu24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV PIXI_ENV=gpu-prod

WORKDIR /code

COPY --from=prod-env /code/.pixi/envs/gpu-prod .pixi/envs/gpu-prod
COPY --from=prod-env /code/scripts/pixi-shell-hook scripts/pixi-shell-hook
COPY --from=builder /odm-runtime/ ./

RUN chmod +x scripts/docker-entrypoint.sh run.sh \
    && bash scripts/docker-entrypoint.sh python3 scripts/smoke.py

ENTRYPOINT ["/code/scripts/docker-entrypoint.sh"]
