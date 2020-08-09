# Prepare TIFs from Parrot Sequoia

The Sequoia captures five images per shot, an RGB JPG and four single band TIFs, but only the JPG contains GPS Exif tags. To use the single-band TIFs for multispectral mapping/analysis it’s useful (not technically required) to have them also contain GPS Exif tags.

## prepare-tifs.sh

This script will copy the GPS Exif tags from the RGB JPG file to the four TIFs of the same shot. It will process all images in the working directory and use the sequence number in the filename to match JPG to TIFs. Therefore it *must* only be applied to one folder from the Sequoia per run.

### Requirements
* [exiv2](https://www.exiv2.org/)
* bash (tested in git-bash on Windows, other shells probably work too)

### Usage
Run the script from a directory of images captured by Sequoia.
```
./prepare-tifs.sh
```

## subfolder-per-band.sh

This script will move images into a subfolder per band.

### Usage
Run the script from a directory of images captured by Sequoia.
```
./prepare-tifs.sh
```
