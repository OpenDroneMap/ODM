import os
import rasterio
from rasterio.io import MemoryFile
import warnings
import numpy as np
import pygltflib

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

    return obj

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

                with MemoryFile() as memfile:
                    with rasterio.open(map_kd, 'r') as src:
                        data = src.read()
                        with memfile.open(driver='JPEG', jpeg_quality=90, count=3, width=src.width, height=src.height, dtype=rasterio.dtypes.uint8) as dst:
                            for b in range(1, min(3, src.count) + 1):
                                # TODO: convert if uint16 or float
                                dst.write(data[b - 1], b)
                    memfile.seek(0)
                    mats[current_mtl] = memfile.read()
    return mats

def paddedBuffer(buf, boundary):
    r = len(buf) % boundary
    if r == 0: 
        return buf 
    pad = boundary - r
    return buf + b'\x00' * pad

def obj2glb(input_obj, output_glb, _info=print):
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
        materials += [pygltflib.Material(pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(baseColorTexture=pygltflib.TextureInfo(index=len(textures) - 1), metallicFactor=0, roughnessFactor=1), 
                        alphaMode=pygltflib.MASK)]

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

    gltf.set_binary_blob(binary)

    _info("Writing...")
    gltf.save(output_glb)
    _info("Wrote %s" % output_glb)

