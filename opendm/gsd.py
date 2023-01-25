import os
import json
import numpy as np
import math
from repoze.lru import lru_cache
from opendm import log
from opendm.shots import get_origin

def rounded_gsd(reconstruction_json, default_value=None, ndigits=0, ignore_gsd=False):
    """
    :param reconstruction_json path to OpenSfM's reconstruction.json
    :return GSD value rounded. If GSD cannot be computed, or ignore_gsd is set, it returns a default value.
    """
    if ignore_gsd:
        return default_value

    gsd = opensfm_reconstruction_average_gsd(reconstruction_json)

    if gsd is not None:
        return round(gsd, ndigits)
    else:
        return default_value


def image_max_size(photos, target_resolution, reconstruction_json, gsd_error_estimate = 0.5, ignore_gsd=False, has_gcp=False):
    """
    :param photos images database
    :param target_resolution resolution the user wants have in cm / pixel
    :param reconstruction_json path to OpenSfM's reconstruction.json
    :param gsd_error_estimate percentage of estimated error in the GSD calculation to set an upper bound on resolution.
    :param ignore_gsd if set to True, simply return the largest side of the largest image in the images database.
    :return A dimension in pixels calculated by taking the image_scale_factor and applying it to the size of the largest image.
        Returned value is never higher than the size of the largest side of the largest image.
    """
    max_width = 0
    max_height = 0
    if ignore_gsd:
        isf = 1.0
    else:
        isf = image_scale_factor(target_resolution, reconstruction_json, gsd_error_estimate, has_gcp=has_gcp)

    for p in photos:
        max_width = max(p.width, max_width)
        max_height = max(p.height, max_height)

    return int(math.ceil(max(max_width, max_height) * isf))

def image_scale_factor(target_resolution, reconstruction_json, gsd_error_estimate = 0.5, has_gcp=False):
    """
    :param target_resolution resolution the user wants have in cm / pixel
    :param reconstruction_json path to OpenSfM's reconstruction.json
    :param gsd_error_estimate percentage of estimated error in the GSD calculation to set an upper bound on resolution.
    :return A down-scale (<= 1) value to apply to images to achieve the target resolution by comparing the current GSD of the reconstruction.
        If a GSD cannot be computed, it just returns 1. Returned scale values are never higher than 1 and are always obtained by dividing by 2 (e.g. 0.5, 0.25, etc.)
    """
    gsd = opensfm_reconstruction_average_gsd(reconstruction_json, use_all_shots=has_gcp)

    if gsd is not None and target_resolution > 0:
        gsd = gsd * (1 + gsd_error_estimate)
        isf = min(1.0, abs(gsd) / target_resolution)
        ret = 0.5
        while ret >= isf:
            ret /= 2.0
        return ret * 2.0
    else:
        return 1.0


def cap_resolution(resolution, reconstruction_json, gsd_error_estimate = 0.1, gsd_scaling = 1.0, ignore_gsd=False,
                   ignore_resolution=False, has_gcp=False):
    """
    :param resolution resolution in cm / pixel
    :param reconstruction_json path to OpenSfM's reconstruction.json
    :param gsd_error_estimate percentage of estimated error in the GSD calculation to set an upper bound on resolution.
    :param gsd_scaling scaling of estimated GSD.
    :param ignore_gsd when set to True, forces the function to just return resolution.
    :param ignore_resolution when set to True, forces the function to return a value based on GSD.
    :return The max value between resolution and the GSD computed from the reconstruction.
        If a GSD cannot be computed, or ignore_gsd is set to True, it just returns resolution. Units are in cm / pixel.
    """
    if ignore_gsd:
        return resolution

    gsd = opensfm_reconstruction_average_gsd(reconstruction_json, use_all_shots=has_gcp or ignore_resolution)

    if gsd is not None:
        gsd = gsd * (1 - gsd_error_estimate) * gsd_scaling
        if gsd > resolution or ignore_resolution:
            log.ODM_WARNING('Maximum resolution set to {} * (GSD - {}%) '
                            '({:.2f} cm / pixel, requested resolution was {:.2f} cm / pixel)'
                            .format(gsd_scaling, gsd_error_estimate * 100, gsd, resolution))
            return gsd
        else:
            return resolution
    else:
        log.ODM_WARNING('Cannot calculate GSD, using requested resolution of {:.2f}'.format(resolution))
        return resolution


@lru_cache(maxsize=None)
def opensfm_reconstruction_average_gsd(reconstruction_json, use_all_shots=False):
    """
    Computes the average Ground Sampling Distance of an OpenSfM reconstruction.
    :param reconstruction_json path to OpenSfM's reconstruction.json
    :return Ground Sampling Distance value (cm / pixel) or None if 
        a GSD estimate cannot be compute
    """
    if not os.path.isfile(reconstruction_json):
        raise IOError(reconstruction_json + " does not exist.")

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
        if use_all_shots or shot.get('gps_dop', 999999) < 999999:
            camera = reconstruction['cameras'][shot['camera']]
            shot_origin = get_origin(shot)
            shot_height = shot_origin[2]
            focal_ratio = camera.get('focal', camera.get('focal_x'))
            if not focal_ratio:
                log.ODM_WARNING("Cannot parse focal values from %s. This is likely an unsupported camera model." % reconstruction_json)
                return None
                
            gsds.append(calculate_gsd_from_focal_ratio(focal_ratio, 
                                                        shot_height - ground_height, 
                                                        camera['width']))
    
    if len(gsds) > 0:
        mean = np.mean(gsds)
        if mean < 0:
            log.ODM_WARNING("Negative GSD estimated, this might indicate a flipped Z-axis.")
        return abs(mean)
    
    return None


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