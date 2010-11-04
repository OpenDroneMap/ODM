#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - running $PMVS"
echo


$PMVS pmvs/ option-0000

echo
echo "  < done - `date`"

exit
