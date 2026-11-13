import numpy as np
from numpy import ndarray
from typing import Tuple
from pyproj import Proj
from opensfm.geo import TopocentricConverter

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


class TopocentricToProj:
    def __init__(self, reflat:float, reflon:float, refalt:float, a_srs:str):
        self.reference = TopocentricConverter(reflat, reflon, refalt)
        self.projection = Proj(a_srs)
        
    def convert_array(self, arr:ndarray, offset_x:float=0, offset_y:float=0):
        x, y, z = arr['X'], arr['Y'], arr['Z']
        easting, northing, alt = self.convert_points(x, y, z, offset_x, offset_y)
        arr['X'] = easting
        arr['Y'] = northing
        arr['Z'] = alt
        return arr
    
    def convert_points(self, x:ndarray, y:ndarray, z:ndarray, offset_x:float=0, offset_y:float=0):
        lat, lon, alt = self.reference.to_lla(x, y, z)
        easting, northing = self.projection(lon, lat)
        return easting - offset_x, northing - offset_y, alt
        
    def convert_obj(self, input_obj:str, output_obj:str, offset_x:float=0, offset_y:float=0):
        with open(input_obj, 'r') as fin:
            with open(output_obj, 'w') as fout:
                lines = fin.readlines()
                for line in lines:
                    if line.startswith("v "):
                        v = np.fromstring(line.strip()[2:] + " 1",  sep=' ', dtype=float)
                        vt = self.convert_points(v[0], v[1], v[2], offset_x, offset_y)
                        fout.write("v " + " ".join(map(str, list(vt))) + '\n')
                    else:
                        fout.write(line)


    
