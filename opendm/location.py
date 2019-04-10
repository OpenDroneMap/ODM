import math
from opendm import log
from pyproj import Proj

def extract_utm_coords(photos, images_path, output_coords_file):
    """
    Create a coordinate file containing the GPS positions of all cameras 
    to be used later in the ODM toolchain for automatic georeferecing
    :param photos ([ODM_Photo]) list of photos
    :param images_path (str) path to dataset images
    :param output_coords_file (str) path to output coordinates file
    :return None
    """
    if len(photos) == 0:
        raise Exception("No input images, cannot create coordinates file of GPS positions")
    
    utm_zone = None
    hemisphere = None
    coords = []
    reference_photo = None
    for photo in photos:
        if photo.latitude is None or photo.longitude is None or photo.altitude is None:
            log.ODM_ERROR("Failed parsing GPS position for %s, skipping" % photo.filename)
            continue
        
        if utm_zone is None:
            utm_zone, hemisphere = get_utm_zone_and_hemisphere_from(photo.longitude, photo.latitude)

        try:
            coord = convert_to_utm(photo.longitude, photo.latitude, photo.altitude, utm_zone, hemisphere)
        except:
            raise Exception("Failed to convert GPS position to UTM for %s" % photo.filename)
        
        coords.append(coord)

    if utm_zone is None:
        raise Exception("No images seem to have GPS information")
        
    # Calculate average
    dx = 0.0
    dy = 0.0
    num = float(len(coords))
    for coord in coords:
        dx += coord[0] / num
        dy += coord[1] / num

    dx = int(math.floor(dx))
    dy = int(math.floor(dy))

    # Open output file
    with open(output_coords_file, "w") as f:
        f.write("WGS84 UTM %s%s\n" % (utm_zone, hemisphere))
        f.write("%s %s\n" % (dx, dy))
        for coord in coords:
            f.write("%s %s %s\n" % (coord[0] - dx, coord[1] - dy, coord[2]))
    

def get_utm_zone_and_hemisphere_from(lon, lat):
    """
    Calculate the UTM zone and hemisphere that a longitude/latitude pair falls on
    :param lon longitude
    :param lat latitude
    :return [utm_zone, hemisphere]
    """
    utm_zone = (int(math.floor((lon + 180.0)/6.0)) % 60) + 1
    hemisphere = 'S' if lat < 0 else 'N'
    return [utm_zone, hemisphere]

def convert_to_utm(lon, lat, alt, utm_zone, hemisphere):
    """
    Convert longitude, latitude and elevation values to UTM
    :param lon longitude
    :param lat latitude
    :param alt altitude
    :param utm_zone UTM zone number
    :param hemisphere one of 'N' or 'S'
    :return [x,y,z] UTM coordinates
    """
    if hemisphere == 'N':
        p = Proj(proj='utm',zone=utm_zone,ellps='WGS84', preserve_units=True)
    else:
        p = Proj(proj='utm',zone=utm_zone,ellps='WGS84', preserve_units=True, south=True)
    
    x,y = p(lon, lat)
    return [x, y, alt]

