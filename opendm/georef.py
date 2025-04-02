import numpy as np
from numpy import ndarray
from typing import Tuple
from pyproj import Proj
import argparse
from opensfm.geo import TopocentricConverter

def parse_pdal_args(pdal_args: dict) -> argparse.Namespace:
    def validate_arg(name: str, data_type: type):
        if name not in pdal_args:
            raise ValueError(f"PDAL arguments should contain {name}.")
        try:
             data_type(pdal_args[name])
        except ValueError:
            raise ValueError(f"PDAL argument {name} should be of type {data_type}.")
        return data_type(pdal_args[name])
    return argparse.Namespace(
        reflat=validate_arg('reflat', float),
        reflon=validate_arg('reflon', float),
        refalt=validate_arg('refalt', float),
        x_offset=validate_arg('x_offset', float),
        y_offset=validate_arg('y_offset', float),
        a_srs=validate_arg('a_srs', str)
    )

def topocentric_to_georef_pdal(ins, outs):
    args = parse_pdal_args(ins)
    outs['X'], outs['Y'], outs['Z'] = topocentric_to_georef(args.reflat, args.reflon, args.refalt, args.a_srs, ins['X'], ins['Y'], ins['Z'], args.x_offset, args.y_offset)
    return True

def topocentric_to_georef(
        reflat: float, 
        reflon: float, 
        refalt: float, 
        a_srs: str, 
        x: ndarray, 
        y: ndarray, 
        z: ndarray,
        x_offset: float = 0,
        y_offset: float = 0,
    ) -> Tuple[ndarray, ndarray, ndarray]:
    reference = TopocentricConverter(reflat, reflon, refalt)
    projection = Proj(a_srs)
    lat, lon, alt = reference.to_lla(x, y, z)
    easting, northing = projection(lon, lat)
    return easting - x_offset, northing - y_offset, alt


    
