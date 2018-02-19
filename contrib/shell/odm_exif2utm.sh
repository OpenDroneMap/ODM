#!/bin/bash
#GPL2 jmenezes ODM extract exif lon/lat project to utm with proj4; 2017-05-28
# line 23  tab bugfix 2017-07-11
# apt-get install exiftool geotiff-bin
if [ $# -lt 2 ]; then
    echo "run inside /images/ directory" 1>&2
    echo $(basename $0)" zone  [S|N] > camera_wgs84utm.txt" 1>&2
    exit 1
fi
Z=$1
case $2 in
s|S) printf "EPSG:327%02d\n" $Z; H=south
;;
n|N) printf "EPSG:326%02d\n" $Z; H=north
;;
*) 
;;
esac
for i in *[jpg,JPG,tif,TIF]; do 
    exiftool $i | grep  GPS | grep  Position | \
    awk -F \: -v img=$i '{ print $2","img }' | tr -d [:blank:] | \
    sed s/deg/d/g | tr \, \\t | awk '{ print $2,$1,$3 }' | \
    proj  -f "%.3f" +proj=utm +zone=$Z +$H +ellps=WGS84 
done | sed s/\ /\\t/g
exit 0
