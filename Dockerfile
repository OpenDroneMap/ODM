# Pixi container layout: https://pixi.prefix.dev/latest/deployment/container/

FROM ghcr.io/prefix-dev/pixi:0.68.1 AS dev

# devcontainer common-utils creates user `odm`; drop default `ubuntu` (uid 1000) if present.
RUN if id "ubuntu" &>/dev/null; then \
        userdel -f -r ubuntu || echo "Failed to delete ubuntu user"; \
    fi

WORKDIR /code

FROM dev AS builder

COPY pixi.toml pixi.lock ./
RUN pixi install --locked

COPY . ./
RUN pixi run build && pixi run test

RUN mkdir -p /odm-runtime/SuperBuild /odm-runtime/scripts \
    && cp -a SuperBuild/install /odm-runtime/SuperBuild/ \
    && cp -a opendm stages /odm-runtime/ \
    && cp run.py run.sh settings.yaml VERSION /odm-runtime/ \
    && cp scripts/docker-entrypoint.sh scripts/smoke.sh /odm-runtime/scripts/

FROM dev AS prod-env

COPY pixi.toml pixi.lock ./
RUN pixi install --locked -e prod \
    && mkdir -p scripts \
    && pixi shell-hook -e prod -s bash > scripts/pixi-shell-hook \
    && rm -rf .pixi/envs/prod/include .pixi/envs/prod/share/{doc,man,info}

FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /code

COPY --from=prod-env /code/.pixi/envs/prod .pixi/envs/prod
COPY --from=prod-env /code/scripts/pixi-shell-hook scripts/pixi-shell-hook
COPY --from=builder /odm-runtime/ ./

RUN chmod +x scripts/docker-entrypoint.sh run.sh scripts/smoke.sh \
    && bash scripts/smoke.sh

ENTRYPOINT ["/code/scripts/docker-entrypoint.sh"]
