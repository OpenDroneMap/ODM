#!/bin/bash
echo "STARTING NEW ODM RUN WITH IMAGES IN OpenDroneMap/images. ALL OTHER PROGRESS WILL BE ERASED."
rm -rf images_resize/
rm -rf odm_georeferencing/
rm -rf odm_meshing/
rm -rf odm_orthophoto/
rm -rf odm_texturing/
rm -rf opensfm/
rm -rf pmvs/
time python run.py --project-path ~/OpenDroneMap/ --odm_meshing-octreeDepth 12 --odm_meshing-maxVertexCount 200000 --min-num-features 5000 --pmvs-level 0  --pmvs-csize 1 --pmvs-wsize 10 

