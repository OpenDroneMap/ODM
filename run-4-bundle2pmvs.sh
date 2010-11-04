#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - running $BUNDLE2PVMS"
echo

$BUNDLE2PVMS list.txt bundle/bundle.out

sed -i $IMAGE_DIR/pmvs/prep_pmvs.sh -e "4c\BUNDLER_BIN_PATH=\"$TOOLS_BIN_PATH\""
sh $IMAGE_DIR/pmvs/prep_pmvs.sh

echo
echo "  < done - `date`"

exit
