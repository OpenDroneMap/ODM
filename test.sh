#!/bin/bash

RUNPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -z "$1" ]; then
	exec python3 -m unittest discover tests "test_$1.py"
else
	exec python3 -m unittest discover tests "test_*.py"
fi
