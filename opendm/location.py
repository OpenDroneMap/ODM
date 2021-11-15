import math
from opendm import log
from pyproj import Proj, Transformer, CRS
from osgeo import osr

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
        if photo.latitude is None or photo.longitude is None:
            log.ODM_WARNING("GPS position not available for %s" % photo.filename)
            continue
        
        if utm_zone is None:
            utm_zone, hemisphere = get_utm_zone_and_hemisphere_from(photo.longitude, photo.latitude)

        try:
            alt = photo.altitude if photo.altitude is not None else 0
            coord = convert_to_utm(photo.longitude, photo.latitude, alt, utm_zone, hemisphere)
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
    
def transform2(from_srs, to_srs, x, y):
    return transformer(from_srs, to_srs).TransformPoint(x, y, 0)[:2]

def transform3(from_srs, to_srs, x, y, z):
    return transformer(from_srs, to_srs).TransformPoint(x, y, z)

def proj_srs_convert(srs):
    """
    Convert a Proj SRS object to osr SRS object
    """
    res = osr.SpatialReference()
    epsg = srs.to_epsg()

    if epsg:
        res.ImportFromEPSG(epsg)
    else:
        proj4 = srs.to_proj4()
        res.ImportFromProj4(proj4)
    
    res.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)

    return res

def transformer(from_srs, to_srs):
    src = proj_srs_convert(from_srs)
    tgt = proj_srs_convert(to_srs)
    return osr.CoordinateTransformation(src, tgt)
    
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

def parse_srs_header(header):
    """
    Parse a header coming from GCP or coordinate file
    :param header (str) line
    :return Proj object
    """
    log.ODM_INFO('Parsing SRS header: %s' % header)
    header = header.strip()
    ref = header.split(' ')

    try:
        if ref[0] == 'WGS84' and ref[1] == 'UTM':
            datum = ref[0]
            utm_pole = (ref[2][len(ref[2]) - 1]).upper()
            utm_zone = int(ref[2][:len(ref[2]) - 1])
            
            proj_args = {
                'zone': utm_zone, 
                'datum': datum
            }

            proj4 = '+proj=utm +zone={zone} +datum={datum} +units=m +no_defs=True'
            if utm_pole == 'S':
                proj4 += ' +south=True'

            srs = CRS.from_proj4(proj4.format(**proj_args))
        elif '+proj' in header:
            srs = CRS.from_proj4(header.strip('\''))
        elif header.lower().startswith("epsg:"):
            srs = CRS.from_epsg(header.lower()[5:])
        else:
            raise RuntimeError('Could not parse coordinates. Bad SRS supplied: %s' % header)
    except RuntimeError as e:
        log.ODM_ERROR('Uh oh! There seems to be a problem with your coordinates/GCP file.\n\n'
                            'The line: %s\n\n'
                            'Is not valid. Projections that are valid include:\n'
                            ' - EPSG:*****\n'
                            ' - WGS84 UTM **(N|S)\n'
                            ' - Any valid proj4 string (for example, +proj=utm +zone=32 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs)\n\n'
                            'Modify your input and try again.' % header)
        raise RuntimeError(e)
    
    return srs