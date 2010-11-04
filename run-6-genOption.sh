#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - running $GENOPTION"
echo

$GENOPTION pmvs/

echo
echo "  < done - `date`"

exit
