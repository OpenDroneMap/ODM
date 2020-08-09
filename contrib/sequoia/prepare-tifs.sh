#!/bin/sh

# Copy GPS Exif tags from IMG_*_RGB.JPG to .TIF images

for rgb in IMG_*_RGB.JPG; do
	seqnum=`echo $rgb | sed -r "s/IMG_[0-9]+_[0-9]+_([0-9]+).*/\1/"`
	echo $seqnum
	exiv2 -PEVk --grep GPS $rgb > $rgb.tags

	for band in GRE NIR RED REG; do
		exiv2 -m $rgb.tags IMG_*_$seqnum\_$band.TIF
	done

	rm $rgb.tags
done
