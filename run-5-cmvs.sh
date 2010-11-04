#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - running $CMVS"
echo

$CMVS pmvs/ 100 4

echo
echo "  < done - `date`"

exit
