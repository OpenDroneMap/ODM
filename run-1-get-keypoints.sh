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

# Create the list of images
find $IMAGE_DIR -maxdepth 1 | egrep ".jpg$" | sort > list_tmp.txt
$EXTRACT_FOCAL list_tmp.txt
cp prepare/list.txt .

echo
echo '[- Extracting keypoints -]'
echo

for d in `ls -1 $IMAGE_DIR | egrep "jpg$"`
do 
    key_file=`echo $d | sed 's/jpg$/key/'`
    pgm_file=`echo $d | sed 's/jpg$/pgm/'`
    jpg_file=`echo $d`

#	SIFT_CMD="$VLSIFT -o $IMAGE_DIR/$key_file -x $IMAGE_DIR/$jpg_file; gzip -f $IMAGE_DIR/$key_file"
	SIFT_CMD="mogrify -format pgm $IMAGE_DIR/$jpg_file; $VLSIFT -v -o $IMAGE_DIR/$key_file $IMAGE_DIR/$pgm_file; rm $IMAGE_DIR/$pgm_file; gzip -f $IMAGE_DIR/$key_file"
#	SIFT_CMD="mogrify -format pgm $IMAGE_DIR/$jpg_file; $SIFT < $IMAGE_DIR/$pgm_file > $IMAGE_DIR/$key_file; rm $IMAGE_DIR/$pgm_file; gzip -f $IMAGE_DIR/$key_file"
	eval $SIFT_CMD
done

exit
