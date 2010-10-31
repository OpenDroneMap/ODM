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

# Run Bundle2PMVS!
echo
echo '[- Running Bundle2PMVS -]'
echo

$BUNDLE2PVMS list.txt bundle/bundle.out

sed -i $IMAGE_DIR/pmvs/prep_pmvs.sh -e "4c\BUNDLER_BIN_PATH=\"$BASE_PATH\""
sh ../pmvs/prep_pmvs.sh

exit
