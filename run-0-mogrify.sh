#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - scaling images"
echo

# Rename ".JPG" to ".jpg"
for d in `ls -1 $IMAGE_DIR | egrep ".JPG$"`
do 
    mv $d `echo $d | sed 's/\.JPG/\.jpg/'`
done

mogrify -resize 1600x1200 -quality 100 *

echo
echo "  < done - `date`"

exit
