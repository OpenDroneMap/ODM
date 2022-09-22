#!/bin/bash
__dirname=$(cd "$(dirname "$0")"; pwd -P)
cd "${__dirname}"

PYTHONPATH=$PYTHONPATH:/code/SuperBuild/install/bin/opensfm python3 orthorectify.py "$@"
