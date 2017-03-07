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
import materials_utils
import subprocess

surfaceShaderType = 'ShaderNodeEmission'
surfaceShaderName = 'Emission'


def main():

    if len(sys.argv) < 5 or sys.argv[-2] != '--':
        sys.exit('Please provide the ODM project path.')

    projectHome = sys.argv[-1]

    bpy.utils.register_module('materials_utils')

    bpy.ops.import_scene.obj(filepath=projectHome +
                             '/odm_texturing/odm_textured_model_geo.obj',
                             axis_forward='Y', axis_up='Z')

    bpy.ops.xps_tools.convert_to_cycles_all()

    model = bpy.data.objects[-1]
    minX = float('inf')
    maxX = float('-inf')
    minY = float('inf')
    maxY = float('-inf')
    minZ = float('inf')
    maxZ = float('-inf')
    for coord in model.bound_box:
        x = coord[0]
        y = coord[1]
        z = coord[2]
        minX = min(x, minX)
        maxX = max(x, maxX)
        minY = min(y, minY)
        maxY = max(y, maxY)
        minZ = min(z, minZ)
        maxZ = max(z, maxZ)

    model.location[2] += (maxZ - minZ)/2

    for m in bpy.data.materials:
        nt = m.node_tree
        nt.nodes.remove(nt.nodes['Color Mult'])
        nt.nodes.remove(nt.nodes['Diffuse BSDF'])
        nt.nodes.new(surfaceShaderType)
        nt.links.new(nt.nodes['Material Output'].inputs[0],
                     nt.nodes[surfaceShaderName].outputs[0])
        nt.links.new(nt.nodes[surfaceShaderName].inputs[0],
                     nt.nodes['Diffuse Texture'].outputs[0])

    blendName = bpy.path.display_name_from_filepath(bpy.data.filepath)
    fileName = projectHome + '/odm_photo/odm_' + blendName
    render = bpy.data.scenes[0].render
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
