import os

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types
from opendm import gsd
from opendm.concurrency import get_max_memory
from opendm.cropper import Cropper
from opendm.cutline import compute_cutline


class ODMOrthoPhotoStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        verbose = '-verbose' if self.params.get('verbose') else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_orthophoto)

        if not io.file_exists(tree.odm_orthophoto_file) or self.rerun():

            # odm_orthophoto definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'log': tree.odm_orthophoto_log,
                'ortho': tree.odm_orthophoto_file,
                'corners': tree.odm_orthophoto_corners,
                'res': 1.0 / (gsd.cap_resolution(self.params.get('resolution'), tree.opensfm_reconstruction, ignore_gsd=args.ignore_gsd) / 100.0),
                'verbose': verbose
            }

            # Have geo coordinates?
            georef = reconstruction.georef

            # Check if the georef object is initialized
            # (during a --rerun this might not be)
            # TODO: we should move this to a more central
            # location (perhaps during the dataset initialization)
            if georef and not georef.utm_east_offset:
                georeferencing_dir = tree.odm_georeferencing if args.use_3dmesh and not args.skip_3dmodel else tree.odm_25dgeoreferencing
                odm_georeferencing_model_txt_geo_file = os.path.join(georeferencing_dir, tree.odm_georeferencing_model_txt_geo)

                if io.file_exists(odm_georeferencing_model_txt_geo_file):
                    georef.extract_offsets(odm_georeferencing_model_txt_geo_file)
                else:
                    log.ODM_WARNING('Cannot read UTM offset from {}. An orthophoto will not be generated.'.format(odm_georeferencing_model_txt_geo_file))


            if georef:
                if args.use_3dmesh:
                    kwargs['model_geo'] = os.path.join(tree.odm_texturing, tree.odm_georeferencing_model_obj_geo)
                else:
                    kwargs['model_geo'] = os.path.join(tree.odm_25dtexturing, tree.odm_georeferencing_model_obj_geo)
            else:
                if args.use_3dmesh:
                    kwargs['model_geo'] = os.path.join(tree.odm_texturing, tree.odm_textured_model_obj)
                else:
                    kwargs['model_geo'] = os.path.join(tree.odm_25dtexturing, tree.odm_textured_model_obj)

            # run odm_orthophoto
            system.run('{bin}/odm_orthophoto -inputFile {model_geo} '
                       '-logFile {log} -outputFile {ortho} -resolution {res} {verbose} '
                       '-outputCornerFile {corners}'.format(**kwargs))

            # Create georeferenced GeoTiff
            geotiffcreated = False

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
                    'tiled': '' if self.params.get('no_tiled') else '-co TILED=yes ',
                    'compress': self.params.get('compress'),
                    'predictor': '-co PREDICTOR=2 ' if self.params.get('compress') in
                                                       ['LZW', 'DEFLATE'] else '',
                    'proj': georef.projection.srs,
                    'bigtiff': self.params.get('bigtiff'),
                    'png': tree.odm_orthophoto_file,
                    'tiff': tree.odm_orthophoto_tif,
                    'log': tree.odm_orthophoto_tif_log,
                    'max_memory': get_max_memory(),
                    'threads': self.params.get('max_concurrency')
                }

                system.run('gdal_translate -a_ullr {ulx} {uly} {lrx} {lry} '
                           '{tiled} '
                           '-co BIGTIFF={bigtiff} '
                           '-co COMPRESS={compress} '
                           '{predictor} '
                           '-co BLOCKXSIZE=512 '
                           '-co BLOCKYSIZE=512 '
                           '-co NUM_THREADS={threads} '
                           '-a_srs \"{proj}\" '
                           '--config GDAL_CACHEMAX {max_memory}% '
                           '{png} {tiff} > {log}'.format(**kwargs))

                bounds_file_path = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.gpkg')
                    
                # Cutline computation, before cropping
                # We want to use the full orthophoto, not the cropped one.
                if args.compute_cutline:
                    compute_cutline(tree.odm_orthophoto_tif, 
                                    bounds_file_path,
                                    os.path.join(tree.odm_orthophoto, "cutline.gpkg"),
                                    self.params.get('max_concurrency'))

                if args.crop > 0:
                    Cropper.crop(bounds_file_path, tree.odm_orthophoto_tif, {
                            'TILED': 'NO' if self.params.get('no_tiled') else 'YES',
                            'COMPRESS': self.params.get('compress'),
                            'PREDICTOR': '2' if self.params.get('compress') in ['LZW', 'DEFLATE'] else '1',
                            'BIGTIFF': self.params.get('bigtiff'),
                            'BLOCKXSIZE': 512,
                            'BLOCKYSIZE': 512,
                            'NUM_THREADS': self.params.get('max_concurrency')
                        })

                if self.params.get('build_overviews'):
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
