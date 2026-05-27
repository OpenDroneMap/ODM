#!/bin/bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "${ROOT}/scripts/pixi-shell-hook" ] \
   && [ -f "${ROOT}/.pixi/envs/prod/bin/python" ]; then
    . "${ROOT}/scripts/pixi-shell-hook"
fi

exec python3 "${ROOT}/run.py" "$@"
