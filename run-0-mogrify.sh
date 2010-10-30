#!/bin/bash

BASE_PATH=$(dirname $(which $0));
IMAGE_DIR="."

EXTRACT_FOCAL=$BASE_PATH/extract_focal.pl
MATCHKEYS=$BASE_PATH/KeyMatchFull
BUNDLER=$BASE_PATH/bundler
BUNDLE2PVMS=$BASE_PATH/Bundle2PMVS
CMVS=$BASE_PATH/cmvs
PMVS=$BASE_PATH/pmvs2
GENOPTION=$BASE_PATH/genOption
SIFT=$BASE_PATH/sift
SIFTFEAT=$BASE_PATH/siftfeat

if [ $# -eq 1 ]
then
    echo "Using directory '$1'"
    IMAGE_DIR=$1
fi

# Rename ".JPG" to ".jpg"
for d in `ls -1 $IMAGE_DIR | egrep ".JPG$"`
do 
    mv $d `echo $d | sed 's/\.JPG/\.jpg/'`
done

echo
echo '[- Scaling images -]'
echo

mogrify -resize 1600x1200 -quality 100 *

exit
