import os
import shutil
import warnings
import numpy as np
from opendm import get_image_size
from opendm import location
from opendm.gcp import GCPFile
from pyproj import CRS
import xmltodict as x2d
from six import string_types

from opendm import log
from opendm import io
from opendm import system
from opendm import context

from opendm.progress import progressbc
from opendm.photo import ODM_Photo

# Ignore warnings about proj information being lost
warnings.filterwarnings("ignore")

class ODM_Reconstruction(object):
    def __init__(self, photos):
        self.photos = photos
        self.georef = None
        self.gcp = None
        self.multi_camera = self.detect_multi_camera()
        self.filter_photos()

    def detect_multi_camera(self):
        """
        Looks at the reconstruction photos and determines if this
        is a single or multi-camera setup.
        """
        band_photos = {}
        band_indexes = {}

        for p in self.photos:
            if not p.band_name in band_photos:
                band_photos[p.band_name] = []
            if not p.band_name in band_indexes:
                band_indexes[p.band_name] = str(p.band_index)

            band_photos[p.band_name].append(p)
            
        bands_count = len(band_photos)
        if bands_count >= 2 and bands_count <= 8:
            # Validate that all bands have the same number of images,
            # otherwise this is not a multi-camera setup
            img_per_band = len(band_photos[p.band_name])
            for band in band_photos:
                if len(band_photos[band]) != img_per_band:
                    log.ODM_ERROR("Multi-camera setup detected, but band \"%s\" (identified from \"%s\") has only %s images (instead of %s), perhaps images are missing or are corrupted. Please include all necessary files to process all bands and try again." % (band, band_photos[band][0].filename, len(band_photos[band]), img_per_band))
                    raise RuntimeError("Invalid multi-camera images")
            
            mc = []
            for band_name in band_indexes:
                mc.append({'name': band_name, 'photos': band_photos[band_name]})
            
            # Sort by band index
            mc.sort(key=lambda x: band_indexes[x['name']])

            return mc

        return None

    def filter_photos(self):
        if not self.multi_camera:
            return # Nothing to do, use all images
        
        else:
            # Sometimes people might try process both RGB + Blue/Red/Green bands
            # because these are the contents of the SD card from a drone (e.g. DJI P4 Multispectral)
            # But we don't want to process both, so we discard the RGB files in favor
            bands = {}
            for b in self.multi_camera:
                bands[b['name'].lower()] = b['name']

            if ('rgb' in bands or 'redgreenblue' in bands) and \
                ('red' in bands and 'green' in bands and 'blue' in bands):
                band_to_remove = bands['rgb'] if 'rgb' in bands else bands['redgreenblue']

                self.multi_camera = [b for b in self.multi_camera if b['name'] != band_to_remove]
                photos_before = len(self.photos)
                self.photos = [p for p in self.photos if p.band_name != band_to_remove]
                photos_after = len(self.photos)

                log.ODM_WARNING("RGB images detected alongside individual Red/Green/Blue images, we will use individual bands (skipping %s images)" % (photos_before - photos_after))

    def is_georeferenced(self):
        return self.georef is not None

    def has_gcp(self):
        return self.is_georeferenced() and self.gcp is not None and self.gcp.exists()

    def georeference_with_gcp(self, gcp_file, output_coords_file, output_gcp_file, output_model_txt_geo, rerun=False):
        if not io.file_exists(output_coords_file) or not io.file_exists(output_gcp_file) or rerun:
            gcp = GCPFile(gcp_file)
            if gcp.exists():
                if gcp.entries_count() == 0:
                    raise RuntimeError("This GCP file does not have any entries. Are the entries entered in the proper format?")

                # Convert GCP file to a UTM projection since the rest of the pipeline
                # does not handle other SRS well.
                rejected_entries = []
                utm_gcp = GCPFile(gcp.create_utm_copy(output_gcp_file, filenames=[p.filename for p in self.photos], rejected_entries=rejected_entries, include_extras=False))
                
                if not utm_gcp.exists():
                    raise RuntimeError("Could not project GCP file to UTM. Please double check your GCP file for mistakes.")
                
                for re in rejected_entries:
                    log.ODM_WARNING("GCP line ignored (image not found): %s" % str(re))
                
                if utm_gcp.entries_count() > 0:
                    log.ODM_INFO("%s GCP points will be used for georeferencing" % utm_gcp.entries_count())
                else:
                    raise RuntimeError("A GCP file was provided, but no valid GCP entries could be used. Note that the GCP file is case sensitive (\".JPG\" is not the same as \".jpg\").")
                
                self.gcp = utm_gcp

                # Compute RTC offsets from GCP points
                x_pos = [p.x for p in utm_gcp.iter_entries()]
                y_pos = [p.y for p in utm_gcp.iter_entries()]
                x_off, y_off = int(np.round(np.mean(x_pos))), int(np.round(np.mean(y_pos)))

                # Create coords file, we'll be using this later
                # during georeferencing
                with open(output_coords_file, 'w') as f:
                    coords_header = gcp.wgs84_utm_zone()
                    f.write(coords_header + "\n")
                    f.write("{} {}\n".format(x_off, y_off))
                    log.ODM_INFO("Generated coords file from GCP: %s" % coords_header)
                
                # Deprecated: This is mostly for backward compatibility and should be
                # be removed at some point
                shutil.copyfile(output_coords_file, output_model_txt_geo)
                log.ODM_INFO("Wrote %s" % output_model_txt_geo)
            else:
                log.ODM_WARNING("GCP file does not exist: %s" % gcp_file)
                return
        else:
            log.ODM_INFO("Coordinates file already exist: %s" % output_coords_file)
            log.ODM_INFO("GCP file already exist: %s" % output_gcp_file)
            self.gcp = GCPFile(output_gcp_file)
        
        self.georef = ODM_GeoRef.FromCoordsFile(output_coords_file)
        return self.georef

    def georeference_with_gps(self, images_path, output_coords_file, output_model_txt_geo, rerun=False):
        try:
            if not io.file_exists(output_coords_file) or rerun:
                location.extract_utm_coords(self.photos, images_path, output_coords_file)
            else:
                log.ODM_INFO("Coordinates file already exist: %s" % output_coords_file)
            
            # Deprecated: This is mostly for backward compatibility and should be
            # be removed at some point
            if not io.file_exists(output_model_txt_geo) or rerun:
                with open(output_coords_file, 'r') as f:
                    with open(output_model_txt_geo, 'w+') as w:
                        w.write(f.readline()) # CRS
                        w.write(f.readline()) # Offset
            else:
                log.ODM_INFO("Model geo file already exist: %s" % output_model_txt_geo)
            
            self.georef = ODM_GeoRef.FromCoordsFile(output_coords_file)
        except:
            log.ODM_WARNING('Could not generate coordinates file. The orthophoto will not be georeferenced.')

        self.gcp = GCPFile(None)
        return self.georef

    def save_proj_srs(self, file):
        # Save proj to file for future use (unless this 
        # dataset is not georeferenced)
        if self.is_georeferenced():
            with open(file, 'w') as f:
                f.write(self.get_proj_srs())

    def get_proj_srs(self):
        if self.is_georeferenced():
            return self.georef.proj4()
    
    def get_proj_offset(self):
        if self.is_georeferenced():
            return (self.georef.utm_east_offset, self.georef.utm_north_offset)
        else:
            return (None, None)

    def get_photo(self, filename):
        for p in self.photos:
            if p.filename == filename:
                return p
    

class ODM_GeoRef(object):
    @staticmethod
    def FromCoordsFile(coords_file):
        # check for coordinate file existence
        if not io.file_exists(coords_file):
            log.ODM_WARNING('Could not find file %s' % coords_file)
            return

        srs = None
        utm_east_offset = None
        utm_north_offset = None

        with open(coords_file) as f:
            # extract reference system and utm zone from first line.
            # We will assume the following format:
            # 'WGS84 UTM 17N' or 'WGS84 UTM 17N \n'
            line = f.readline().rstrip()
            srs = location.parse_srs_header(line)

            # second line is a northing/easting offset
            line = f.readline().rstrip()
            utm_east_offset, utm_north_offset = map(float, line.split(" "))

        return ODM_GeoRef(srs, utm_east_offset, utm_north_offset)

    def __init__(self, srs, utm_east_offset, utm_north_offset):
        self.srs = srs
        self.utm_east_offset = utm_east_offset
        self.utm_north_offset = utm_north_offset
        self.transform = []

    def proj4(self):
        return self.srs.to_proj4()
    
    def utm_offset(self):
        return (self.utm_east_offset, self.utm_north_offset)
    
class ODM_Tree(object):
    def __init__(self, root_path, gcp_file = None, geo_file = None):
        # root path to the project
        self.root_path = io.absolute_path_file(root_path)
        self.input_images = os.path.join(self.root_path, 'images')

        # modules paths

        # here are defined where all modules should be located in
        # order to keep track all files al directories during the
        # whole reconstruction process.
        self.dataset_raw = os.path.join(self.root_path, 'images')
        self.opensfm = os.path.join(self.root_path, 'opensfm')
        self.openmvs = os.path.join(self.opensfm, 'undistorted', 'openmvs')
        self.odm_meshing = os.path.join(self.root_path, 'odm_meshing')
        self.odm_texturing = os.path.join(self.root_path, 'odm_texturing')
        self.odm_25dtexturing = os.path.join(self.root_path, 'odm_texturing_25d')
        self.odm_georeferencing = os.path.join(self.root_path, 'odm_georeferencing')
        self.odm_filterpoints = os.path.join(self.root_path, 'odm_filterpoints')
        self.odm_orthophoto = os.path.join(self.root_path, 'odm_orthophoto')
        self.odm_report = os.path.join(self.root_path, 'odm_report')

        # important files paths

        # benchmarking
        self.benchmarking = os.path.join(self.root_path, 'benchmark.txt')
        self.dataset_list = os.path.join(self.root_path, 'img_list.txt')

        # opensfm
        self.opensfm_image_list = os.path.join(self.opensfm, 'image_list.txt')
        self.opensfm_reconstruction = os.path.join(self.opensfm, 'reconstruction.json')
        self.opensfm_reconstruction_nvm = os.path.join(self.opensfm, 'undistorted/reconstruction.nvm')
        self.opensfm_geocoords_reconstruction = os.path.join(self.opensfm, 'reconstruction.geocoords.json')
        self.opensfm_topocentric_reconstruction = os.path.join(self.opensfm, 'reconstruction.topocentric.json')

        # OpenMVS
        self.openmvs_model = os.path.join(self.openmvs, 'scene_dense_dense_filtered.ply')

        # filter points
        self.filtered_point_cloud = os.path.join(self.odm_filterpoints, "point_cloud.ply")

        # odm_meshing
        self.odm_mesh = os.path.join(self.odm_meshing, 'odm_mesh.ply')
        self.odm_meshing_log = os.path.join(self.odm_meshing, 'odm_meshing_log.txt')
        self.odm_25dmesh = os.path.join(self.odm_meshing, 'odm_25dmesh.ply')
        self.odm_25dmeshing_log = os.path.join(self.odm_meshing, 'odm_25dmeshing_log.txt')

        # texturing
        self.odm_textured_model_obj = 'odm_textured_model_geo.obj'

        # odm_georeferencing
        self.odm_georeferencing_coords = os.path.join(
            self.odm_georeferencing, 'coords.txt')
        self.odm_georeferencing_gcp = gcp_file or io.find('gcp_list.txt', self.root_path)
        self.odm_georeferencing_gcp_utm = os.path.join(self.odm_georeferencing, 'gcp_list_utm.txt')
        self.odm_geo_file = geo_file or io.find('geo.txt', self.root_path)
        
        self.odm_georeferencing_proj = 'proj.txt'
        self.odm_georeferencing_model_txt_geo = os.path.join(
            self.odm_georeferencing, 'odm_georeferencing_model_geo.txt')
        self.odm_georeferencing_xyz_file = os.path.join(
            self.odm_georeferencing, 'odm_georeferenced_model.csv')
        self.odm_georeferencing_model_laz = os.path.join(
            self.odm_georeferencing, 'odm_georeferenced_model.laz')
        self.odm_georeferencing_model_las = os.path.join(
            self.odm_georeferencing, 'odm_georeferenced_model.las')

        # odm_orthophoto
        self.odm_orthophoto_render = os.path.join(self.odm_orthophoto, 'odm_orthophoto_render.tif')
        self.odm_orthophoto_tif = os.path.join(self.odm_orthophoto, 'odm_orthophoto.tif')
        self.odm_orthophoto_corners = os.path.join(self.odm_orthophoto, 'odm_orthophoto_corners.txt')
        self.odm_orthophoto_log = os.path.join(self.odm_orthophoto, 'odm_orthophoto_log.txt')
        self.odm_orthophoto_tif_log = os.path.join(self.odm_orthophoto, 'gdal_translate_log.txt')

        # tiles
        self.orthophoto_tiles = os.path.join(self.root_path, "orthophoto_tiles")

        # Split-merge 
        self.submodels_path = os.path.join(self.root_path, 'submodels')

        # Tiles
        self.entwine_pointcloud = self.path("entwine_pointcloud")
        self.ogc_tiles = self.path("3d_tiles")

    def path(self, *args):
        return os.path.join(self.root_path, *args)


class ODM_Stage:
    def __init__(self, name, args, progress=0.0, **params):
        self.name = name
        self.args = args
        self.progress = progress
        self.params = params
        if self.params is None:
            self.params = {}
        self.next_stage = None
        self.prev_stage = None

    def connect(self, stage):
        self.next_stage = stage
        stage.prev_stage = self
        return stage

    def rerun(self):
        """
        Does this stage need to be rerun?
        """
        return (self.args.rerun is not None and self.args.rerun == self.name) or \
                     (self.args.rerun_all) or \
                     (self.args.rerun_from is not None and self.name in self.args.rerun_from)
    
    def run(self, outputs = {}):
        start_time = system.now_raw()
        log.logger.log_json_stage_run(self.name, start_time)

        log.ODM_INFO('Running %s stage' % self.name)
        
        self.process(self.args, outputs)

        # The tree variable should always be populated at this point
        if outputs.get('tree') is None:
            raise Exception("Assert violation: tree variable is missing from outputs dictionary.")

        if self.args.time:
            system.benchmark(start_time, outputs['tree'].benchmarking, self.name)

        log.ODM_INFO('Finished %s stage' % self.name)
        self.update_progress_end()

        # Last stage?
        if self.args.end_with == self.name or self.args.rerun == self.name:
            log.ODM_INFO("No more stages to run")
            return

        # Run next stage?
        elif self.next_stage is not None:
            self.next_stage.run(outputs)

    def delta_progress(self):
        if self.prev_stage:
            return max(0.0, self.progress - self.prev_stage.progress)
        else:
            return max(0.0, self.progress)
    
    def previous_stages_progress(self):
        if self.prev_stage:
            return max(0.0, self.prev_stage.progress)
        else:
            return 0.0

    def update_progress_end(self):
        self.update_progress(100.0)

    def update_progress(self, progress):
        progress = max(0.0, min(100.0, progress))
        progressbc.send_update(self.previous_stages_progress() + 
                              (self.delta_progress() / 100.0) * float(progress))

    def last_stage(self):
        if self.next_stage:
            return self.next_stage.last_stage()
        else:
            return self
        

    def process(self, args, outputs):
        raise NotImplementedError

