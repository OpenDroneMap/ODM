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

    if len(vertices) > len(uvs):
        # Pad with empty UV coordinates
        add_uvs = len(vertices) - len(uvs)
        uvs += [[0,0]] * add_uvs

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
                    with rasterio.open(map_kd, 'r') as r:
                        mats[current_mtl] = r.read()
                        # TODO: copy code from export-rgb (webodm)
    return mats


def obj2glb(input_obj, output_glb, _info=print):
    obj = load_obj(input_obj, _info=_info)
    
    vertices = obj['vertices']
    uvs = obj['uvs']
    normals = obj['normals']

    vertices_blob = vertices.tobytes()
    uvs_blob = uvs.tobytes()

    faces = obj['faces']['material0000']
    material = obj['materials']['material0000']
    print(material.shape)
    print(material.)
    exit(1)
    # TODO: all faces

    faces = np.array(faces, dtype=np.uint32)

    indices = faces[:,0:3].flatten()
    uv_indices = faces[:,3:6].flatten()

    if faces.shape[1] == 9:
        normal_indices = faces[:,6:9].flatten()
    else:
        normal_indices = None

    #faces_blob = faces.tobytes()

    indices_blob = indices.tobytes()
    uv_indices_blob = uv_indices.tobytes()

    binary = vertices_blob + uvs_blob + indices_blob + uv_indices_blob

    gltf = pygltflib.GLTF2(
        scene=0,
        scenes=[pygltflib.Scene(nodes=[0])],
        nodes=[pygltflib.Node(mesh=0)],
        meshes=[
            pygltflib.Mesh(
                primitives=[
                    pygltflib.Primitive(
                        attributes=pygltflib.Attributes(POSITION=0, TEXCOORD_0=1), indices=2, material=0
                    )
                ]
            )
        ],
        materials=[

        ],
        textures=[
            pygltflib.Texture(source=0, sampler=0)
        ],
        images=[
            pygltflib.Image(bufferView=0, mimeType="image/png") # TODO: use JPG
        ],
        accessors=[
            pygltflib.Accessor(
                bufferView=0,
                componentType=pygltflib.FLOAT,
                count=len(vertices),
                type=pygltflib.VEC3,
                max=vertices.max(axis=0).tolist(),
                min=vertices.min(axis=0).tolist(),
            ),
            pygltflib.Accessor(
                bufferView=1,
                componentType=pygltflib.FLOAT,
                count=len(uvs),
                type=pygltflib.VEC2,
                max=uvs.max(axis=0).tolist(),
                min=uvs.min(axis=0).tolist(),
            ),
            pygltflib.Accessor(
                bufferView=2,
                componentType=pygltflib.UNSIGNED_INT,
                count=len(indices),
                type=pygltflib.SCALAR,
                max=[int(indices.max())],
                min=[int(indices.min())],
            ),
            pygltflib.Accessor(
                bufferView=3,
                componentType=pygltflib.UNSIGNED_INT,
                count=len(uv_indices),
                type=pygltflib.SCALAR,
                max=[int(uv_indices.max())],
                min=[int(uv_indices.min())],
            ),
        ],
        bufferViews=[
            pygltflib.BufferView(
                buffer=0,
                byteLength=len(vertices_blob),
                target=pygltflib.ARRAY_BUFFER,
            ),
            pygltflib.BufferView(
                buffer=0,
                byteOffset=len(vertices_blob),
                byteLength=len(uvs_blob),
                target=pygltflib.ARRAY_BUFFER,
            ),
            pygltflib.BufferView(
                buffer=0,
                byteOffset=len(vertices_blob) + len(uvs_blob),
                byteLength=len(indices_blob),
                target=pygltflib.ELEMENT_ARRAY_BUFFER,
            ),
            pygltflib.BufferView(
                buffer=0,
                byteOffset=len(vertices_blob) + len(uvs_blob) + len(indices_blob),
                byteLength=len(uv_indices_blob),
                target=pygltflib.ELEMENT_ARRAY_BUFFER,
            ),
            pygltflib.BufferView(
                buffer=0,
                byteOffset=len(vertices_blob) + len(uvs_blob) + len(indices_blob) + len(uv_indices_blob),
                byteLength=len(texture_blob),
                target=pygltflib.ARRAY_BUFFER
            )
        ],
        buffers=[
            pygltflib.Buffer(
                byteLength=len(binary)
            )
        ],
    )

    gltf.set_binary_blob(binary)

    gltf.save(output_glb)
    print("OK")

