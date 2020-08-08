#!/bin/sh

# Rename - remove timestamp
for img in IMG_*.*; do
	newname=`echo $img | sed -r "s/IMG_[0-9]+_[0-9]+_//"`
	mv $img $newname
done

# Copy GPS Exif tags from *RGB.JPG to .TIF images
for rgb in *_RGB.JPG; do
	for band in GRE NIR RED REG; do
		tif=`echo $rgb | sed s/_RGB.JPG/_$band.TIF/`
		exiv2 -PEVk --grep GPS $rgb > $rgb.tags
		exiv2 -m $rgb.tags $tif
		rm $rgb.tags
	done
done

# Move into subfolder per band
for band in RGB GRE NIR RED REG; do
	mkdir $band
	mv *_$band.* $band/
done
