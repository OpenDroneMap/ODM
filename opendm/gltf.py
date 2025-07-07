import os
import rasterio
from rasterio.io import MemoryFile
import warnings
import numpy as np
import pygltflib
from opendm import system
from opendm import io
from opendm import log

warnings.filterwarnings("ignore", category=rasterio.errors.NotGeoreferencedWarning)


def load_obj(obj_path, _info=print):
    if not os.path.isfile(obj_path):
        raise IOError("Cannot open %s" % obj_path)

    obj_base_path = os.path.dirname(os.path.abspath(obj_path))
    obj = {
        'materials': {},
    }
    vertices = []
    uvs = []
    normals = []

    faces = {}
    current_material = "_"

    with open(obj_path) as f:
        _info("Loading %s" % obj_path)

        for line in f:
            if line.startswith("mtllib "):
                # Materials
                mtl_file = "".join(line.split()[1:]).strip()
                obj['materials'].update(load_mtl(mtl_file, obj_base_path, _info=_info))
            elif line.startswith("v "):
                # Vertices
                vertices.append(list(map(float, line.split()[1:4])))
            elif line.startswith("vt "):
                # UVs
                uvs.append(list(map(float, line.split()[1:3])))
            elif line.startswith("vn "):
                normals.append(list(map(float, line.split()[1:4])))
            elif line.startswith("usemtl "):
                mtl_name = "".join(line.split()[1:]).strip()
                if not mtl_name in obj['materials']:
                    raise Exception("%s material is missing" % mtl_name)

                current_material = mtl_name
            elif line.startswith("f "):
                if current_material not in faces:
                    faces[current_material] = []
                
                a,b,c = line.split()[1:]

                if a.count("/") == 2:
                    av, at, an = map(int, a.split("/")[0:3])
                    bv, bt, bn = map(int, b.split("/")[0:3])
                    cv, ct, cn = map(int, c.split("/")[0:3])

                    faces[current_material].append((av - 1, bv - 1, cv - 1, at - 1, bt - 1, ct - 1, an - 1, bn - 1, cn - 1)) 
                else:
                    av, at = map(int, a.split("/")[0:2])
                    bv, bt = map(int, b.split("/")[0:2])
                    cv, ct = map(int, c.split("/")[0:2])
                    faces[current_material].append((av - 1, bv - 1, cv - 1, at - 1, bt - 1, ct - 1)) 

    obj['vertices'] = np.array(vertices, dtype=np.float32)
    obj['uvs'] = np.array(uvs, dtype=np.float32)
    obj['normals'] = np.array(normals, dtype=np.float32)
    obj['faces'] = faces

    obj['materials'] = convert_materials_to_jpeg(obj['materials'])

    return obj

def convert_materials_to_jpeg(materials):

    min_value = 0
    value_range = 0
    skip_conversion = False

    for mat in materials:
        image = materials[mat]

        # Stop here, assuming all other materials are also uint8
        if image.dtype == np.uint8:
            skip_conversion = True
            break

        # Find common min/range values
        try:
            data_range = np.iinfo(image.dtype)
            min_value = min(min_value, 0)
            value_range = max(value_range, float(data_range.max) - float(data_range.min))
        except ValueError:
            # For floats use the actual range of the image values
            min_value = min(min_value, float(image.min()))
            value_range = max(value_range, float(image.max()) - min_value)
    
    if value_range == 0:
        value_range = 255 # Should never happen

    for mat in materials:
        image = materials[mat]

        if not skip_conversion:
            image = image.astype(np.float32)
            image -= min_value
            image *= 255.0 / value_range
            np.around(image, out=image)
            image[image > 255] = 255
            image[image < 0] = 0
            image = image.astype(np.uint8)

        with MemoryFile() as memfile:
            bands, h, w = image.shape
            bands = min(3, bands)
            with memfile.open(driver='JPEG', jpeg_quality=90, count=bands, width=w, height=h, dtype=rasterio.dtypes.uint8) as dst:
                for b in range(1, min(3, bands) + 1):
                    dst.write(image[b - 1], b)
            memfile.seek(0)
            materials[mat] = memfile.read()

    return materials

def load_mtl(mtl_file, obj_base_path, _info=print):
    mtl_file = os.path.join(obj_base_path, mtl_file)

    if not os.path.isfile(mtl_file):
        raise IOError("Cannot open %s" % mtl_file)
    
    mats = {}
    current_mtl = ""

    with open(mtl_file) as f:
        for line in f:
            if line.startswith("newmtl "):
                current_mtl = "".join(line.split()[1:]).strip()
            elif line.startswith("map_Kd ") and current_mtl:
                map_kd_filename = "".join(line.split()[1:]).strip()
                map_kd = os.path.join(obj_base_path, map_kd_filename)
                if not os.path.isfile(map_kd):
                    raise IOError("Cannot open %s" % map_kd)
                
                _info("Loading %s" % map_kd_filename)
                with rasterio.open(map_kd, 'r') as src:
                    mats[current_mtl] = src.read()
    return mats

def paddedBuffer(buf, boundary):
    r = len(buf) % boundary
    if r == 0: 
        return buf 
    pad = boundary - r
    return buf + b'\x00' * pad

def obj2glb(input_obj, output_glb, rtc=(None, None), draco_compression=True, _info=print):
    _info("Converting %s --> %s" % (input_obj, output_glb))
    obj = load_obj(input_obj, _info=_info)

    vertices = obj['vertices']
    uvs = obj['uvs']
    # Flip Y
    uvs = (([0, 1] - (uvs * [0, 1])) + uvs * [1, 0]).astype(np.float32)
    normals = obj['normals']

    binary = b''
    accessors = []
    bufferViews = []
    primitives = []
    materials = []
    textures = []
    images = []

    bufOffset = 0
    def addBufferView(buf, target=None):
        nonlocal bufferViews, bufOffset
        bufferViews += [pygltflib.BufferView(
            buffer=0,
            byteOffset=bufOffset,
            byteLength=len(buf),
            target=target,
        )]
        bufOffset += len(buf)
        return len(bufferViews) - 1

    for material in obj['faces'].keys():
        faces = obj['faces'][material]
        faces = np.array(faces, dtype=np.uint32)

        prim_vertices = vertices[faces[:,0:3].flatten()]
        prim_uvs = uvs[faces[:,3:6].flatten()]

        if faces.shape[1] == 9:
            prim_normals = normals[faces[:,6:9].flatten()]
            normals_blob = prim_normals.tobytes()
        else:
            prim_normals = None
            normals_blob = None

        vertices_blob = prim_vertices.tobytes()
        uvs_blob = prim_uvs.tobytes()

        binary += vertices_blob + uvs_blob
        if normals_blob is not None:
            binary += normals_blob
        
        verticesBufferView = addBufferView(vertices_blob, pygltflib.ARRAY_BUFFER)
        uvsBufferView = addBufferView(uvs_blob, pygltflib.ARRAY_BUFFER)
        normalsBufferView = None
        if normals_blob is not None:
            normalsBufferView = addBufferView(normals_blob, pygltflib.ARRAY_BUFFER)
        
        accessors += [
            pygltflib.Accessor(
                bufferView=verticesBufferView,
                componentType=pygltflib.FLOAT,
                count=len(prim_vertices),
                type=pygltflib.VEC3,
                max=prim_vertices.max(axis=0).tolist(),
                min=prim_vertices.min(axis=0).tolist(),
            ),
            pygltflib.Accessor(
                bufferView=uvsBufferView,
                componentType=pygltflib.FLOAT,
                count=len(prim_uvs),
                type=pygltflib.VEC2,
                max=prim_uvs.max(axis=0).tolist(),
                min=prim_uvs.min(axis=0).tolist(),
            ),
        ]

        if prim_normals is not None:
            accessors += [
                pygltflib.Accessor(
                    bufferView=normalsBufferView,
                    componentType=pygltflib.FLOAT,
                    count=len(prim_normals),
                    type=pygltflib.VEC3,
                    max=prim_normals.max(axis=0).tolist(),
                    min=prim_normals.min(axis=0).tolist(),
                )
            ]

        primitives += [pygltflib.Primitive(
                attributes=pygltflib.Attributes(POSITION=verticesBufferView, TEXCOORD_0=uvsBufferView, NORMAL=normalsBufferView), material=len(primitives)
            )]

    for material in obj['faces'].keys():
        texture_blob = paddedBuffer(obj['materials'][material], 4)
        binary += texture_blob
        textureBufferView = addBufferView(texture_blob)

        images += [pygltflib.Image(bufferView=textureBufferView, mimeType="image/jpeg")]
        textures += [pygltflib.Texture(source=len(images) - 1, sampler=0)]

        mat = pygltflib.Material(pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(baseColorTexture=pygltflib.TextureInfo(index=len(textures) - 1), metallicFactor=0, roughnessFactor=1), 
                alphaMode=pygltflib.OPAQUE)
        mat.extensions = {
            'KHR_materials_unlit': {}
        }
        materials += [mat]

    gltf = pygltflib.GLTF2(
        scene=0,
        scenes=[pygltflib.Scene(nodes=[0])],
        nodes=[pygltflib.Node(mesh=0)],
        meshes=[pygltflib.Mesh(
                primitives=primitives
            )],
        materials=materials,
        textures=textures,
        samplers=[pygltflib.Sampler(magFilter=pygltflib.LINEAR, minFilter=pygltflib.LINEAR)],
        images=images,
        accessors=accessors,
        bufferViews=bufferViews,
        buffers=[pygltflib.Buffer(byteLength=len(binary))],
    )

    gltf.extensionsRequired = ['KHR_materials_unlit']
    gltf.extensionsUsed = ['KHR_materials_unlit']

    if rtc != (None, None) and len(rtc) >= 2:
        gltf.extensionsUsed.append('CESIUM_RTC')
        gltf.extensions = {
            'CESIUM_RTC': {
                'center': [float(rtc[0]), float(rtc[1]), 0.0]
            }
        }

    gltf.set_binary_blob(binary)

    _info("Writing...")
    gltf.save(output_glb)
    _info("Wrote %s" % output_glb)

    if draco_compression:
        _info("Compressing with draco")
        try:
            compressed_glb = io.related_file_path(output_glb, postfix="_compressed")
            system.run('draco_transcoder -i "{}" -o "{}" -qt 16 -qp 16'.format(output_glb, compressed_glb))
            if os.path.isfile(compressed_glb) and os.path.isfile(output_glb):
                os.remove(output_glb)
                os.rename(compressed_glb, output_glb)
        except Exception as e:
            log.ODM_WARNING("Cannot compress GLB with draco: %s" % str(e))
            

