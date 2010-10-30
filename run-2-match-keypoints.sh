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

# Match images (can take a while)
echo
echo '[- Matching keypoints (this can take a while) -]'
echo

sed 's/\.jpg$/\.key/' $IMAGE_DIR/list_tmp.txt > $IMAGE_DIR/list_keys.txt

echo $MATCHKEYS $IMAGE_DIR/list_keys.txt $IMAGE_DIR/matches.init.txt
$MATCHKEYS $IMAGE_DIR/list_keys.txt $IMAGE_DIR/matches.init.txt

exit
