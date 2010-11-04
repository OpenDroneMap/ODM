#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - running $MATCHKEYS (will take some time ...)"
echo

sed 's/\.jpg$/\.key/' $IMAGE_DIR/list_tmp.txt > $IMAGE_DIR/list_keys.txt

echo $MATCHKEYS $IMAGE_DIR/list_keys.txt $IMAGE_DIR/matches.init.txt
$MATCHKEYS $IMAGE_DIR/list_keys.txt $IMAGE_DIR/matches.init.txt

echo
echo "  < done - `date`"

exit
