#!/bin/bash

script_dir=$(dirname $0)
. $script_dir/defs.sh

echo
echo "  - running $VLSIFT (will take some time ...)"
echo

# Create the list of images
find $IMAGE_DIR -maxdepth 1 | egrep ".jpg$" | sort > list_tmp.txt
$EXTRACT_FOCAL list_tmp.txt
cp prepare/list.txt .

rm -f match_jobs.txt

for d in `ls -1 $IMAGE_DIR | egrep "jpg$"`
do 
    key_file=`echo $d | sed 's/jpg$/key/'`
    pgm_file=`echo $d | sed 's/jpg$/pgm/'`
    base_file=`echo $d | sed 's/\.jpg$//'`
    jpg_file=`echo $d`

	VLSIFT_CMD="mogrify -format pgm $IMAGE_DIR/$jpg_file; $VLSIFT -o $IMAGE_DIR/$key_file.tmp $IMAGE_DIR/$pgm_file; rm $IMAGE_DIR/$pgm_file; perl $VLSIFT_TO_LOWESIFT $base_file; rm $IMAGE_DIR/$key_file.tmp; gzip -f $IMAGE_DIR/$key_file"	
	SIFT_CMD="mogrify -format pgm $IMAGE_DIR/$jpg_file; $SIFT < $IMAGE_DIR/$pgm_file > $IMAGE_DIR/$key_file; rm $IMAGE_DIR/$pgm_file; gzip -f $IMAGE_DIR/$key_file"

	echo $SIFT_CMD >> match_jobs.txt

##	eval $SIFT_CMD
done

$PARALLEL -j+0 < match_jobs.txt

wait

echo
echo "  < done - `date`"

exit
