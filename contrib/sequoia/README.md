# Prepare TIFs from Parrot Sequoia

The Sequoia captures five images per shot, an RGB JPG and four single band TIFs, but only the JPG contains GPS Exif tags. To use the single-band TIFs for multispectral mapping/analysis it’s useful (not technically required) to have them also contain GPS Exif tags.

This script will copy the GPS Exif tags from the RGB JPG file to the four TIFs of the same shot.

## Requirements
* [exiv2](https://www.exiv2.org/)

## Usage
Run the script from a directory of images captured by Sequoia.
```
./prepare-tifs.sh
```
