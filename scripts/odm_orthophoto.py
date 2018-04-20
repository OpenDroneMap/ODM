import ecto, os
from psutil import virtual_memory

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types
from opendm.cropper import Cropper


class ODMOrthoPhotoCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("resolution", 'Orthophoto ground resolution in pixels/meter', 20)
        params.declare("t_srs", 'Target SRS', None)
        params.declare("no_tiled", 'Do not tile tiff', False)
        params.declare("compress", 'Compression type', 'DEFLATE')
        params.declare("bigtiff", 'Make BigTIFF orthophoto', 'IF_SAFER')
        params.declare("build_overviews", 'Build overviews', False)
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Orthophoto Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        reconstruction = inputs.reconstruction
        verbose = '-verbose' if self.params.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_orthophoto)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_orthophoto') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_orthophoto' in args.rerun_from)

        if not io.file_exists(tree.odm_orthophoto_file) or rerun_cell:

            # odm_orthophoto definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'log': tree.odm_orthophoto_log,
                'ortho': tree.odm_orthophoto_file,
                'corners': tree.odm_orthophoto_corners,
                'res': self.params.resolution,
                'verbose': verbose
            }

            # Have geo coordinates?
            if reconstruction.georef:  # io.file_exists(tree.odm_georeferencing_coords):
                if args.use_25dmesh:
                    kwargs['model_geo'] = os.path.join(tree.odm_25dtexturing, tree.odm_georeferencing_model_obj_geo)
                else:
                    kwargs['model_geo'] = os.path.join(tree.odm_texturing, tree.odm_georeferencing_model_obj_geo)
            else:
                if args.use_25dmesh:
                    kwargs['model_geo'] = os.path.join(tree.odm_25dtexturing, tree.odm_textured_model_obj)
                else:
                    kwargs['model_geo'] = os.path.join(tree.odm_texturing, tree.odm_textured_model_obj)

            # run odm_orthophoto
            system.run('{bin}/odm_orthophoto -inputFile {model_geo} '
                       '-logFile {log} -outputFile {ortho} -resolution {res} {verbose} '
                       '-outputCornerFile {corners}'.format(**kwargs))

            # Create georeferenced GeoTiff
            geotiffcreated = False
            georef = reconstruction.georef

            if georef and georef.projection and georef.utm_east_offset and georef.utm_north_offset:
                ulx = uly = lrx = lry = 0.0
                with open(tree.odm_orthophoto_corners) as f:
                    for lineNumber, line in enumerate(f):
                        if lineNumber == 0:
                            tokens = line.split(' ')
                            if len(tokens) == 4:
                                ulx = float(tokens[0]) + \
                                    float(georef.utm_east_offset)
                                lry = float(tokens[1]) + \
                                    float(georef.utm_north_offset)
                                lrx = float(tokens[2]) + \
                                    float(georef.utm_east_offset)
                                uly = float(tokens[3]) + \
                                    float(georef.utm_north_offset)
                log.ODM_INFO('Creating GeoTIFF')

                kwargs = {
                    'ulx': ulx,
                    'uly': uly,
                    'lrx': lrx,
                    'lry': lry,
                    'tiled': '' if self.params.no_tiled else '-co TILED=yes ',
                    'compress': self.params.compress,
                    'predictor': '-co PREDICTOR=2 ' if self.params.compress in
                                                       ['LZW', 'DEFLATE'] else '',
                    'proj': georef.projection.srs,
                    'bigtiff': self.params.bigtiff,
                    'png': tree.odm_orthophoto_file,
                    'tiff': tree.odm_orthophoto_tif,
                    'log': tree.odm_orthophoto_tif_log,
                    'max_memory': max(5, (100 - virtual_memory().percent) / 2)
                }

                system.run('gdal_translate -a_ullr {ulx} {uly} {lrx} {lry} '
                           '{tiled} '
                           '-co BIGTIFF={bigtiff} '
                           '-co COMPRESS={compress} '
                           '{predictor} '
                           '-co BLOCKXSIZE=512 '
                           '-co BLOCKYSIZE=512 '
                           '-co NUM_THREADS=ALL_CPUS '
                           '-a_srs \"{proj}\" '
                           '--config GDAL_CACHEMAX {max_memory}% '
                           '{png} {tiff} > {log}'.format(**kwargs))

                if args.crop > 0:
                    shapefile_path = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.shp')
                    Cropper.crop(shapefile_path, tree.odm_orthophoto_tif, {
                            'TILED': 'NO' if self.params.no_tiled else 'YES',
                            'COMPRESS': self.params.compress,
                            'PREDICTOR': '2' if self.params.compress in ['LZW', 'DEFLATE'] else '1',
                            'BIGTIFF': self.params.bigtiff,
                            'BLOCKXSIZE': 512,
                            'BLOCKYSIZE': 512,
                            'NUM_THREADS': 'ALL_CPUS'
                        })

                if self.params.build_overviews:
                    log.ODM_DEBUG("Building Overviews")
                    kwargs = {
                        'orthophoto': tree.odm_orthophoto_tif,
                        'log': tree.odm_orthophoto_gdaladdo_log
                    }
                    # Run gdaladdo
                    system.run('gdaladdo -ro -r average '
                               '--config BIGTIFF_OVERVIEW IF_SAFER '
                               '--config COMPRESS_OVERVIEW JPEG '
                               '{orthophoto} 2 4 8 16 > {log}'.format(**kwargs))

                geotiffcreated = True
            if not geotiffcreated:
                log.ODM_WARNING('No geo-referenced orthophoto created due '
                                'to missing geo-referencing or corner coordinates.')

        else:
            log.ODM_WARNING('Found a valid orthophoto in: %s' % tree.odm_orthophoto_file)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Orthophoto')

        log.ODM_INFO('Running ODM OrthoPhoto Cell - Finished')
        return ecto.OK if args.end_with != 'odm_orthophoto' else ecto.QUIT
