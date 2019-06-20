import glob
import os
from opendm import log
from opendm import location
from pyproj import CRS

class GCPFile:
    def __init__(self, gcp_path):
        self.gcp_path = gcp_path
        self.entries = []
        self.raw_srs = ""
        self.srs = None
        self.read()
    
    def read(self):
        if self.exists():
            with open(self.gcp_path, 'r') as f:
                contents = f.read().strip()
    
            lines = map(str.strip, contents.split('\n'))
            if lines:
                self.raw_srs = lines[0] # SRS
                self.srs = location.parse_srs_header(self.raw_srs)

                for line in lines[1:]:
                    if line != "" and line[0] != "#":
                        parts = line.split()
                        if len(parts) >= 6:
                            self.entries.append(line)                          
                        else:
                            log.ODM_WARNING("Malformed GCP line: %s" % line)

    def entries_dict(self):
        for entry in self.entries:
            parts = entry.split()
            x, y, z, px, py, filename = parts[:6]
            extras = " ".join(parts[6:])
            yield {
                'x': x,
                'y': y,
                'z': z,
                'px': px,
                'py': py,
                'filename': filename,
                'extras': extras
            }
    
    def entry_dict_to_s(self, entry):
        return "{x} {y} {z} {px} {py} {filename} {extras}".format(**entry).rstrip()
    
    def exists(self):
        return self.gcp_path and os.path.exists(self.gcp_path)

    def wgs84_utm_zone(self):
        """
        Finds the UTM zone where the first point of the GCP falls into
        :return utm zone string valid for a coordinates header
        """
        if len(self.entries) > 0:
            point = list(self.entries_dict())[0]
            longlat = CRS.from_epsg("4326")
            lat, lon = location.transform(self.srs, longlat, point['x'], point['y'])
            utm_zone, hemisphere = location.get_utm_zone_and_hemisphere_from(lon, lat)
            return "WGS84 UTM %s%s" % (utm_zone, hemisphere)

    def make_filtered_copy(self, gcp_file_output, images_dir, min_images=3):
        """
        Creates a new GCP file from an existing GCP file includes
        only the points that reference images existing in the images_dir directory.
        If less than min_images images are referenced, no GCP copy is created.
        :return gcp_file_output if successful, None if no output file was created.
        """
        if not self.exists() or not os.path.exists(images_dir):
            return None
        
        if os.path.exists(gcp_file_output):
            os.remove(gcp_file_output)

        files = map(os.path.basename, glob.glob(os.path.join(images_dir, "*")))

        output = [self.raw_srs]
        files_found = 0
        
        for entry in self.entries_dict():
            if entry['filename'] in files:
                output.append(self.entry_dict_to_s(entry))
                files_found += 1

        if files_found >= min_images:
            with open(gcp_file_output, 'w') as f:
                f.write('\n'.join(output) + '\n')

            return gcp_file_output
