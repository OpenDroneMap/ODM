#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
. "${ROOT}/scripts/docker-activate.sh"
# shellcheck disable=SC1091
. "${ROOT}/scripts/odm-env.sh"
exec python3 "${ROOT}/run.py" "$@"
