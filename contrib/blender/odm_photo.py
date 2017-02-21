#!/usr/bin/env python

# Renders a photo.
# To generate a 360 panoramic photo:
# blender -b photo_360.blend --python odm_photo.py -- <project-path>
# To generate a 360 3D panoramic photo:
# blender -b photo_vr.blend --python odm_photo.py -- <project-path>
# NB: argument order matters!

import sys
import bpy
import materials_utils

surfaceShaderType = 'ShaderNodeEmission'
surfaceShaderName = 'Emission'


def main():

    if len(sys.argv) < 5 or sys.argv[-2] != '--':
        sys.exit('Please provide the ODM project path.')

    projectHome = sys.argv[-1]

    bpy.utils.register_module('materials_utils')

    bpy.ops.import_scene.obj(filepath=projectHome +
                             '/odm_texturing/odm_textured_model_geo.obj',
                             axis_forward='Y', axis_up='-Z')

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

    model.location[2] += minZ

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
    bpy.data.scenes['Scene'].render.filepath = projectHome +
    '/odm_photo/odm_' + blendName + '.jpg'
    bpy.ops.render.render(write_still=True)


if __name__ == '__main__':
    main()
