#!/bin/bash

RUNPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${RUNPATH}/scripts/odm-env.sh"
exec python3 "${RUNPATH}/run.py" "$@"
