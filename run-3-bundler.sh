#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - running $BUNDLER"
echo

mkdir bundle
rm -f $IMAGE_DIR/options.txt

echo "--match_table matches.init.txt" >> options.txt
echo "--output bundle.out" >> options.txt
echo "--output_all bundle_" >> options.txt
echo "--output_dir bundle" >> options.txt
echo "--variable_focal_length" >> options.txt
echo "--use_focal_estimate" >> options.txt
echo "--constrain_focal" >> options.txt
echo "--constrain_focal_weight 0.0001" >> options.txt
echo "--estimate_distortion" >> options.txt
echo "--run_bundle" >> options.txt

rm -f $IMAGE_DIR/constraints.txt
rm -f $IMAGE_DIR/pairwise_scores.txt
$BUNDLER $IMAGE_DIR/list.txt --options_file $IMAGE_DIR/options.txt > $IMAGE_DIR/bundle/out

echo
echo "  < done - `date`"

exit
