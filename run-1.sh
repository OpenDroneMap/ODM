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

	SIFT_CMD="$SIFTFEAT -o $IMAGE_DIR/$key_file -x $IMAGE_DIR/$jpg_file; gzip -f $IMAGE_DIR/$key_file"
#	SIFT_CMD="mogrify -format pgm $IMAGE_DIR/$jpg_file; $SIFT < $IMAGE_DIR/$pgm_file > $IMAGE_DIR/$key_file; rm $IMAGE_DIR/$pgm_file; gzip -f $IMAGE_DIR/$key_file"
	eval $SIFT_CMD
done

# Match images (can take a while)
echo
echo '[- Matching keypoints (this can take a while) -]'
echo

sed 's/\.jpg$/\.key/' $IMAGE_DIR/list_tmp.txt > $IMAGE_DIR/list_keys.txt

echo $MATCHKEYS $IMAGE_DIR/list_keys.txt $IMAGE_DIR/matches.init.txt
$MATCHKEYS $IMAGE_DIR/list_keys.txt $IMAGE_DIR/matches.init.txt

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

# Run Bundler!
echo
echo '[- Running Bundler -]'
echo

rm -f $IMAGE_DIR/constraints.txt
rm -f $IMAGE_DIR/pairwise_scores.txt
$BUNDLER $IMAGE_DIR/list.txt --options_file $IMAGE_DIR/options.txt > $IMAGE_DIR/bundle/out

# Run Bundle2PMVS!
echo
echo '[- Running Bundle2PMVS -]'
echo

$BUNDLE2PVMS list.txt bundle/bundle.out

# Run prep_pmvs!
echo
echo '[- Running prep_pmvs -]'
echo

sed -i $IMAGE_DIR/pmvs/prep_pmvs.sh -e "4c\BUNDLER_BIN_PATH=\"$BASE_PATH\""
sh pmvs/prep_pmvs.sh

# Run cmvs!
echo
echo '[- Running cmvs -]'
echo

$CMVS pmvs/ 20

# Run pmvs!
echo
echo '[- Running genOption -]'
echo

$GENOPTION pmvs/

# Run prep_pmvs!
echo
echo '[- Running pmvs -]'
echo

$PMVS pmvs/ option-0000
#sh pmvs/pmvs.sh

echo
echo '[- Done -]'
echo