#!/bin/bash

BASE_PATH=$(dirname $(which $0));

sh $BASE_PATH/run-0-mogrify.sh
sh $BASE_PATH/run-1-get-keypoints.sh
sh $BASE_PATH/run-2-match-keypoints.sh
sh $BASE_PATH/run-3-bundler.sh
sh $BASE_PATH/run-4-bundle2pmvs.sh
sh $BASE_PATH/run-5-cmvs.sh
sh $BASE_PATH/run-6-genOption.sh
sh $BASE_PATH/run-7-pmvs.sh

exit
