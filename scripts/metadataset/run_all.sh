#!/usr/bin/env bash

set -e

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

python $DIR/setup.py $1
python $DIR/run_matching.py $1
python $DIR/split.py $1
python $DIR/run_reconstructions.py $1
python $DIR/align.py $1
python $DIR/run_dense.py $1
