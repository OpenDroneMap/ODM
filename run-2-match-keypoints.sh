#!/bin/bash

BASE_PATH=$(dirname $(which $0));
IMAGE_DIR="."

EXTRACT_FOCAL=$BASE_PATH/bin/extract_focal.pl
MATCHKEYS=$BASE_PATH/bin/KeyMatchFull
BUNDLER=$BASE_PATH/bin/bundler
BUNDLE2PVMS=$BASE_PATH/bin/Bundle2PMVS
CMVS=$BASE_PATH/bin/cmvs
PMVS=$BASE_PATH/bin/pmvs2
GENOPTION=$BASE_PATH/bin/genOption
SIFT=$BASE_PATH/bin/sift
SIFTFEAT=$BASE_PATH/siftfeat
VLSIFT=$BASE_PATH/bin/vlsift

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
