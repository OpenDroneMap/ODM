#!/bin/bash

RUNPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${RUNPATH}/run.py" "$@"
