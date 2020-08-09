#!/bin/sh

# Move into subfolder per band
for band in RGB GRE NIR RED REG; do
	mkdir $band
	mv IMG_*_$band.* $band/
done
