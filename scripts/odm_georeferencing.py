import ecto
import csv
import os

from opendm import io
from opendm import log
from opendm import types
from opendm import system
from opendm import context


class ODMGeoreferencingCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("gcp_file", 'path to the file containing the ground control '
                            'points used for georeferencing.The file needs to '
                            'be on the following line format: \neasting '
                            'northing height pixelrow pixelcol imagename', 'gcp_list.txt')
        params.declare("img_size", 'image size used in calibration', 2400)
        params.declare("use_exif", 'use exif', False)
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("photos", "list of ODMPhoto's", [])
        inputs.declare("reconstruction", "list of ODMReconstructions", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # find a file in the root directory
        def find(file, dir):
            for root, dirs, files in os.walk(dir):
                return '/'.join((root, file)) if file in files else None

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Georeferencing Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        gcpfile = io.join_paths(tree.root_path, self.params.gcp_file) \
            if self.params.gcp_file else find('gcp_list.txt', tree.root_path)
        geocreated = True
        verbose = '-verbose' if self.params.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_georeferencing)

        # in case a gcp file it's not provided, let's try to generate it using
        # images metadata. Internally calls jhead.
        log.ODM_DEBUG(self.params.gcp_file)
        if not self.params.gcp_file: # and \
        #   not io.file_exists(tree.odm_georeferencing_coords):
            
            log.ODM_WARNING('No coordinates file. '
                            'Generating coordinates file: %s'
                            % tree.odm_georeferencing_coords)

            # odm_georeference definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'imgs': tree.dataset_resize,
                'imgs_list': tree.opensfm_bundle_list,
                'coords': tree.odm_georeferencing_coords,
                'log': tree.odm_georeferencing_utm_log,
                'verbose': verbose
            }

            # run UTM extraction binary
            extract_utm = system.run_and_return('{bin}/odm_extract_utm -imagesPath {imgs}/ '
                       '-imageListFile {imgs_list} -outputCoordFile {coords} {verbose} '
                       '-logFile {log}'.format(**kwargs))

            if extract_utm != '':
                log.ODM_WARNING('Could not generate coordinates file. '
                                'Ignore if there is a GCP file. Error: %s'
                                % extract_utm)


        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_georeferencing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_georeferencing' in args.rerun_from)

        if not io.file_exists(tree.odm_georeferencing_model_obj_geo) or \
           not io.file_exists(tree.odm_georeferencing_model_ply_geo) or rerun_cell:

            # odm_georeference definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'bundle': tree.opensfm_bundle,
                'imgs': tree.dataset_resize,
                'imgs_list': tree.opensfm_bundle_list,
                'model': tree.odm_textured_model_obj,
                'log': tree.odm_georeferencing_log,
                'coords': tree.odm_georeferencing_coords,
                'pc_geo': tree.odm_georeferencing_model_ply_geo,
                'geo_sys': tree.odm_georeferencing_model_txt_geo,
                'model_geo': tree.odm_georeferencing_model_obj_geo,
                'size': self.params.img_size,
                'gcp': gcpfile,
                'verbose': verbose

            }
            if args.use_opensfm_pointcloud:
                kwargs['pc'] = tree.opensfm_model
            else:
                kwargs['pc'] = tree.pmvs_model

            # Check to see if the GCP file exists

            if not self.params.use_exif and (self.params.gcp_file or find('gcp_list.txt', tree.root_path)):
                log.ODM_INFO('Found %s' % gcpfile)
                try:
                    system.run('{bin}/odm_georef -bundleFile {bundle} -imagesPath {imgs} -imagesListPath {imgs_list} '
                               '-bundleResizedTo {size} -inputFile {model} -outputFile {model_geo} '
                               '-inputPointCloudFile {pc} -outputPointCloudFile {pc_geo} {verbose} '
                               '-logFile {log} -georefFileOutputPath {geo_sys} -gcpFile {gcp} '
                               '-outputCoordFile {coords}'.format(**kwargs))
                except Exception:
                    log.ODM_EXCEPTION('Georeferencing failed. ')
                    return ecto.QUIT
            elif io.file_exists(tree.odm_georeferencing_coords):
                log.ODM_INFO('Running georeferencing with generated coords file.')
                system.run('{bin}/odm_georef -bundleFile {bundle} -inputCoordFile {coords} '
                           '-inputFile {model} -outputFile {model_geo} '
                           '-inputPointCloudFile {pc} -outputPointCloudFile {pc_geo} {verbose} '
                           '-logFile {log} -georefFileOutputPath {geo_sys}'.format(**kwargs))
            else:
                log.ODM_WARNING('Georeferencing failed. Make sure your '
                                'photos have geotags in the EXIF or you have '
                                'provided a GCP file. ')
                geocreated = False # skip the rest of the georeferencing

            if geocreated:
                # update images metadata
                geo_ref = types.ODM_GeoRef()
                geo_ref.parse_coordinate_system(tree.odm_georeferencing_coords)

                for idx, photo in enumerate(self.inputs.photos):
                    geo_ref.utm_to_latlon(tree.odm_georeferencing_latlon, photo, idx)

                # convert ply model to LAS reference system
                geo_ref.convert_to_las(tree.odm_georeferencing_model_ply_geo,
                                       tree.odm_georeferencing_pdal)

                # XYZ point cloud output
                log.ODM_INFO("Creating geo-referenced CSV file (XYZ format, can be used with GRASS to create DEM)")
                with open(tree.odm_georeferencing_xyz_file, "wb") as csvfile:
                    csvfile_writer = csv.writer(csvfile, delimiter=",")
                    reachedpoints = False
                    with open(tree.odm_georeferencing_model_ply_geo) as f:
                        for lineNumber, line in enumerate(f):
                            if reachedpoints:
                                tokens = line.split(" ")
                                csv_line = [float(tokens[0])+geo_ref.utm_east_offset,
                                            float(tokens[1])+geo_ref.utm_north_offset,
                                            tokens[2]]
                                csvfile_writer.writerow(csv_line)
                            if line.startswith("end_header"):
                                reachedpoints = True
                csvfile.close()

        else:
            log.ODM_WARNING('Found a valid georeferenced model in: %s'
                            % tree.odm_georeferencing_model_ply_geo)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Georeferencing')

        log.ODM_INFO('Running ODM Georeferencing Cell - Finished')
        return ecto.OK if args.end_with != 'odm_georeferencing' else ecto.QUIT
