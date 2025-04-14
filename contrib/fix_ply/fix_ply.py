import os
from plyfile import PlyData, PlyElement, PlyProperty
import open3d as o3d
import numpy as np


def pcd_ascii_to_binary_ply(_ply_file: str, _binary_ply: str) -> None:
    """ Converts a .ply saved as ASCII format to a .ply saved with binary format and properties compatible with ODM"""

    ply_data: PlyData = PlyData.read(_ply_file)

    new_elements: list[PlyElement] = []

    for element in ply_data.elements:
        if 'scalar_views' in element.data.dtype.names:
            new_dtype: list[tuple[str, str]] = []
            for name in element.data.dtype.names:
                if name == 'scalar_views':
                    new_dtype.append(('views', 'u1'))
                else:
                    new_dtype.append((name, element.data.dtype[name]))

            new_data = np.empty(element.data.shape, dtype=new_dtype)

            for name in element.data.dtype.names:
                if name == 'scalar_views':
                    new_data['views'] = element.data[name].astype('u1')
                else:
                    new_data[name] = element.data[name]

            new_element = PlyElement.describe(new_data, element.name)
        else:
            new_element = PlyElement.describe(element.data, element.name)

        new_elements.append(new_element)

    new_ply_data = PlyData(new_elements, text=False)

    with open(_binary_ply, 'wb') as f:
        new_ply_data.write(f)


def view_point_cloud(_input_ply: str) -> None:
    pcd = o3d.io.read_point_cloud(_input_ply)
    o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':

    # Parameters
    base: str = os.path.join('D:\\', '01_droneo', 'DJI_202407311545_024', 'odm_filterpoints')
    ply_file: str = os.path.join(base, 'point_cloud_ascii.ply')
    binary_ply_file: str = os.path.join(base, 'point_cloud.ply')

    if not os.path.isfile(ply_file):
        raise "File doesn't exist"

    pcd_ascii_to_binary_ply(ply_file, binary_ply_file)

    view_point_cloud(binary_ply_file)
