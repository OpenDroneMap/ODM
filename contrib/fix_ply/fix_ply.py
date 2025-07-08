import os
import logging
from plyfile import PlyData, PlyElement
import numpy as np

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def pcd_ascii_to_binary_ply(ply_file: str, binary_ply: str) -> None:
    """Converts ASCII PLY to binary, ensuring 'views' is present and of type uchar.
       Raises ValueError if neither 'scalar_views' nor 'views' is found.
    """

    try:
        logging.info(f"Reading ASCII PLY file: {ply_file}")
        ply_data: PlyData = PlyData.read(ply_file)
    except FileNotFoundError:
        logging.error(f"File not found: {ply_file}")
        return
    except Exception as e:
        logging.error(f"Error reading PLY file: {e}")
        return

    new_elements: list[PlyElement] = []

    for element in ply_data.elements:
        new_data = element.data.copy()

        if 'scalar_views' in element.data.dtype.names:
            new_data['views'] = new_data['scalar_views'].astype('u1')
            del new_data['scalar_views']
        elif 'views' in element.data.dtype.names:
            new_data['views'] = new_data['views'].astype('u1')
        else:
            raise ValueError(f"Neither 'scalar_views' nor 'views' found - did you import them when opened the file in CloudCompare?")


        new_element = PlyElement.describe(new_data, element.name)
        new_elements.append(new_element)

    new_ply_data = PlyData(new_elements, text=False)

    try:
        logging.info(f"Writing binary PLY file: {binary_ply}")
        new_ply_data.write(binary_ply)
    except Exception as e:
        logging.error(f"Error writing PLY file: {e}")
        return

    logging.info("PLY conversion complete.")


if __name__ == '__main__':

    # Parameters
    base: str = os.path.dirname(os.path.abspath(__file__))
    ply_file: str = os.path.join(base, 'point_cloud_ascii.ply')
    binary_ply_file: str = os.path.join(base, 'point_cloud.ply')

    if not os.path.exists(ply_file):
        logging.error(f"Input file not found: {ply_file}")
        exit(1) # Exit with error code

    try:
        pcd_ascii_to_binary_ply(ply_file, binary_ply_file)
    except ValueError as e:
        logging.error(f"PLY conversion failed: {e}")
        exit(1) # Exit with error code to indicate failure
