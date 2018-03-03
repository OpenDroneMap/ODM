#!/usr/bin/env bash

RUNPATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/../..
export PYTHONPATH=$RUNPATH:$RUNPATH/SuperBuild/install/lib/python2.7/dist-packages:$RUNPATH/SuperBuild/src/opensfm:$PYTHONPATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RUNPATH/SuperBuild/install/lib

set -e

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

python $DIR/setup.py "$@"
python $DIR/run_matching.py $1
python $DIR/split.py $1
python $DIR/run_reconstructions.py $1
python $DIR/align.py $1
python $DIR/run_dense.py $1
python $DIR/merge.py $1
