#!/usr/bin/env python

# Renders a photo.
# ExifTool must be on your PATH.
# To generate a 360 panoramic photo:
# blender -b photo_360.blend --python odm_photo.py -- <project-path>
# To generate a 360 3D panoramic photo:
# blender -b photo_vr.blend --python odm_photo.py -- <project-path>
# NB: argument order matters!

import sys
import bpy
import subprocess
from common import loadMesh


def main():

    if len(sys.argv) < 5 or sys.argv[-2] != '--':
        sys.exit('Please provide the ODM project path.')

    projectHome = sys.argv[-1]

    loadMesh(projectHome +
             '/odm_texturing/odm_textured_model_geo.obj')

    blendName = bpy.path.display_name_from_filepath(bpy.data.filepath)
    fileName = projectHome + '/odm_photo/odm_' + blendName
    render = bpy.data.scenes['Scene'].render
    render.filepath = fileName
    bpy.ops.render.render(write_still=True)

    width = render.resolution_x
    height = render.resolution_y
    if(render.use_multiview):
        writeExif(fileName+render.views[0].file_suffix+'.jpg', width, height)
        writeExif(fileName+render.views[1].file_suffix+'.jpg', width, height)
    else:
        writeExif(fileName+'.jpg', width, height)


def writeExif(fileName, width, height):
    w = str(width)
    h = str(height)

    subprocess.run(['exiftool',
                    '-overwrite_original',
                    '-CroppedAreaImageWidthPixels=' + w,
                    '-CroppedAreaImageHeightPixels=' + h,
                    '-FullPanoWidthPixels=' + w,
                    '-FullPanoHeightPixels=' + h,
                    '-CroppedAreaLeftPixels=0',
                    '-CroppedAreaTopPixels=0',
                    '-ProjectionType=equirectangular',
                    '-UsePanoramaViewer=True',
                    '-PoseHeadingDegrees=0',
                    '-LargestValidInteriorRectLeft=0',
                    '-LargestValidInteriorRectTop=0',
                    '-LargestValidInteriorRectWidth=' + w,
                    '-LargestValidInteriorRectHeight=' + h,
                    fileName])


if __name__ == '__main__':
    main()
