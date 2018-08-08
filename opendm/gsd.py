import os
import json
import numpy as np

def opensfm_reconstruction_average_gsd(reconstruction_json):
    """
    Computes the average Ground Sampling Distance of an OpenSfM reconstruction.
    :param reconstruction_json path to OpenSfM's reconstruction.json
    :return Ground Sampling Distance value (cm / pixel) or None if 
        a GSD estimate cannot be compute
    """
    if not os.path.isfile(reconstruction_json):
        raise FileNotFoundError(reconstruction_json + " does not exist.")

    with open(reconstruction_json) as f:
        data = json.load(f)

    # Calculate median height from sparse reconstruction
    reconstruction = data[0]
    point_heights = []

    for pointId in reconstruction['points']:
        point = reconstruction['points'][pointId]
        point_heights.append(point['coordinates'][2])

    ground_height = np.median(point_heights)

    gsds = []
    for shotImage in reconstruction['shots']:
        shot = reconstruction['shots'][shotImage]
        if shot['gps_dop'] < 999999:
            camera = reconstruction['cameras'][shot['camera']]

            shot_height = shot['translation'][2]
            focal_ratio = camera['focal']

            gsds.append(calculate_gsd_from_focal_ratio(focal_ratio, 
                                                        shot_height - ground_height, 
                                                        camera['width']))
    
    return np.mean(gsds) if len(gsds) > 0 else None

def calculate_gsd(sensor_width, flight_height, focal_length, image_width):
    """
    :param sensor_width in millimeters
    :param flight_height in meters
    :param focal_length in millimeters
    :param image_width in pixels
    :return Ground Sampling Distance

    >>> round(calculate_gsd(13.2, 100, 8.8, 5472), 2)
    2.74
    >>> calculate_gsd(13.2, 100, 0, 2000)
    >>> calculate_gsd(13.2, 100, 8.8, 0)
    """
    if sensor_width != 0:
        return calculate_gsd_from_focal_ratio(focal_length / sensor_width, 
                                                flight_height, 
                                                image_width)
    else:
        return None

def calculate_gsd_from_focal_ratio(focal_ratio, flight_height, image_width):
    """
    :param focal_ratio focal length (mm) / sensor_width (mm)
    :param flight_height in meters
    :param image_width in pixels
    :return Ground Sampling Distance
    """
    if focal_ratio == 0 or image_width == 0:
        return None
    
    return ((flight_height * 100) / image_width) / focal_ratio