#!/usr/bin/env bash
set -euo pipefail

ninja="${CONDA_PREFIX}/bin/ninja"
if [ ! -x "${ninja}" ]; then
    ninja="${CONDA_PREFIX}/Library/bin/ninja"
fi
if [ ! -x "${ninja}" ]; then
    echo "ninja not found under CONDA_PREFIX=${CONDA_PREFIX}" >&2
    exit 1
fi

cmake -S SuperBuild -B SuperBuild/build -G Ninja \
    -DCMAKE_MAKE_PROGRAM="${ninja}" \
    -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build SuperBuild/build
