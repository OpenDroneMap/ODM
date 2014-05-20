# Script for preparing images and calibration data 
#   for Yasutaka Furukawa's PMVS system

BUNDLER_BIN_PATH= # Edit this line before running
if [ "$BUNDLER_BIN_PATH" == "" ] ; then echo Please edit prep_pmvs.sh to specify the path to the  bundler binaries.; exit; fi
# Apply radial undistortion to the images
$BUNDLER_BIN_PATH/RadialUndistort list.txt bundle/bundle.out pmvs

# Create directory structure
mkdir -p pmvs/txt/
mkdir -p pmvs/visualize/
mkdir -p pmvs/models/

# Copy and rename files
mv pmvs/./kermit000.rd.jpg pmvs/visualize/00000000.jpg
mv pmvs/00000000.txt pmvs/txt/
mv pmvs/./kermit001.rd.jpg pmvs/visualize/00000001.jpg
mv pmvs/00000001.txt pmvs/txt/
mv pmvs/./kermit002.rd.jpg pmvs/visualize/00000002.jpg
mv pmvs/00000002.txt pmvs/txt/
mv pmvs/./kermit003.rd.jpg pmvs/visualize/00000003.jpg
mv pmvs/00000003.txt pmvs/txt/
mv pmvs/./kermit005.rd.jpg pmvs/visualize/00000004.jpg
mv pmvs/00000004.txt pmvs/txt/
mv pmvs/./kermit007.rd.jpg pmvs/visualize/00000005.jpg
mv pmvs/00000005.txt pmvs/txt/
mv pmvs/./kermit008.rd.jpg pmvs/visualize/00000006.jpg
mv pmvs/00000006.txt pmvs/txt/
mv pmvs/./kermit009.rd.jpg pmvs/visualize/00000007.jpg
mv pmvs/00000007.txt pmvs/txt/
mv pmvs/./kermit010.rd.jpg pmvs/visualize/00000008.jpg
mv pmvs/00000008.txt pmvs/txt/

echo "Running Bundle2Vis to generate vis.dat
"
$BUNDLER_BIN_PATH/Bundle2Vis pmvs/bundle.rd.out pmvs/vis.dat



echo @@ Sample command for running pmvs:
echo "   pmvs2 pmvs/ pmvs_options.txt"
echo "    - or - "
echo "   use Dr. Yasutaka Furukawa's view clustering algorithm to generate a set of options files"
