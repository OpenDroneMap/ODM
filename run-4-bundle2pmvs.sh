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

# Run Bundle2PMVS!
echo
echo '[- Running Bundle2PMVS -]'
echo

$BUNDLE2PVMS list.txt bundle/bundle.out

sed -i $IMAGE_DIR/pmvs/prep_pmvs.sh -e "4c\BUNDLER_BIN_PATH=\"$BASE_PATH\""
sh pmvs/prep_pmvs.sh#!/bin/bash

exit
