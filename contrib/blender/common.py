import bpy

def loadMesh(file):

    bpy.utils.register_module('materials_utils')

    bpy.ops.import_scene.obj(filepath=file,
                             axis_forward='Y',
                             axis_up='Z')

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

    surfaceShaderType = 'ShaderNodeEmission'
    surfaceShaderName = 'Emission'

    for m in bpy.data.materials:
        nt = m.node_tree
        nt.nodes.remove(nt.nodes['Color Mult'])
        nt.nodes.remove(nt.nodes['Diffuse BSDF'])
        nt.nodes.new(surfaceShaderType)
        nt.links.new(nt.nodes['Material Output'].inputs[0],
                     nt.nodes[surfaceShaderName].outputs[0])
        nt.links.new(nt.nodes[surfaceShaderName].inputs[0],
                     nt.nodes['Diffuse Texture'].outputs[0])
