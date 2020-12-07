import os
from opendm import log
from opendm import location
from pyproj import CRS

class GeoFile:
    def __init__(self, geo_path):
        self.geo_path = geo_path
        self.entries = {}
        self.srs = None

        with open(self.geo_path, 'r') as f:
            contents = f.read().strip()

        lines = list(map(str.strip, contents.split('\n')))
        if lines:
            self.raw_srs = lines[0]  # SRS
            self.srs = location.parse_srs_header(self.raw_srs)
            longlat = CRS.from_epsg("4326")

            for line in lines[1:]:
                if line != "" and line[0] != "#":
                    parts = line.split()
                    if len(parts) >= 3:
                        i = 3
                        filename = parts[0]
                        x, y = [float(p) for p in parts[1:3]]
                        z = float(parts[3]) if len(parts) >= 4 else None

                        # Always convert coordinates to WGS84
                        if z is not None:
                            x, y, z = location.transform3(self.srs, longlat, x, y, z)
                        else:
                            x, y = location.transform2(self.srs, longlat, x, y)

                        omega = phi = kappa = None

                        if len(parts) >= 7:
                            omega, phi, kappa = [float(p) for p in parts[4:7]]
                            i = 7

                        horizontal_accuracy = vertical_accuracy = None
                        if len(parts) >= 9:
                            horizontal_accuracy,vertical_accuracy = [float(p) for p in parts[7:9]]
                            i = 9

                        extras = " ".join(parts[i:])
                        self.entries[filename] = GeoEntry(filename, x, y, z,
                                                        omega, phi, kappa,
                                                        horizontal_accuracy, vertical_accuracy, 
                                                        extras)
                    else:
                        log.ODM_WARNING("Malformed geo line: %s" % line)
    
    def get_entry(self, filename):
        return self.entries.get(filename)


class GeoEntry:
    def __init__(self, filename, x, y, z, omega=None, phi=None, kappa=None, horizontal_accuracy=None, vertical_accuracy=None, extras=None):
        self.filename = filename
        self.x = x
        self.y = y
        self.z = z
        self.omega = omega
        self.phi = phi
        self.kappa = kappa
        self.horizontal_accuracy = horizontal_accuracy
        self.vertical_accuracy = vertical_accuracy
        self.extras = extras

    def __str__(self):
        return "{} ({} {} {}) ({} {} {}) ({} {}) {}".format(self.filename, 
                                             self.x, self.y, self.z,
                                             self.omega, self.phi, self.kappa,
                                             self.horizontal_accuracy, self.vertical_accuracy,
                                             self.extras).rstrip()
    
    def position_string(self):
        return "{} {} {}".format(self.x, self.y, self.z)
