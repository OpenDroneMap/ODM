#!/bin/bash

RUNPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "${RUNPATH}/scripts/odm-env.sh"

if [ ! -z "$1" ]; then
	exec python3 -m unittest discover tests "test_$1.py"
else
	exec python3 -m unittest discover tests "test_*.py"
fi
