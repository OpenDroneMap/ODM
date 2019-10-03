import os

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types
from opendm import gsd
from opendm import orthophoto
from opendm.concurrency import get_max_memory
from opendm.cropper import Cropper
from opendm.cutline import compute_cutline


class ODMOrthoPhotoStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        verbose = '-verbose' if args.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_orthophoto)

        if not io.file_exists(tree.odm_orthophoto_file) or self.rerun():

            # odm_orthophoto definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'log': tree.odm_orthophoto_log,
                'ortho': tree.odm_orthophoto_file,
                'corners': tree.odm_orthophoto_corners,
                'res': 1.0 / (gsd.cap_resolution(args.orthophoto_resolution, tree.opensfm_reconstruction, ignore_gsd=args.ignore_gsd) / 100.0),
                'verbose': verbose
            }

            # Check if the georef object is initialized
            # (during a --rerun this might not be)
            # TODO: this should be moved to a more central location?
            if reconstruction.is_georeferenced() and not reconstruction.georef.valid_utm_offsets():
                georeferencing_dir = tree.odm_georeferencing if args.use_3dmesh and not args.skip_3dmodel else tree.odm_25dgeoreferencing
                odm_georeferencing_model_txt_geo_file = os.path.join(georeferencing_dir, tree.odm_georeferencing_model_txt_geo)

                if io.file_exists(odm_georeferencing_model_txt_geo_file):
                    reconstruction.georef.extract_offsets(odm_georeferencing_model_txt_geo_file)
                else:
                    log.ODM_WARNING('Cannot read UTM offset from {}. An orthophoto will not be generated.'.format(odm_georeferencing_model_txt_geo_file))

            if reconstruction.is_georeferenced():
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

            if reconstruction.is_georeferenced() and reconstruction.georef.valid_utm_offsets():
                ulx = uly = lrx = lry = 0.0
                with open(tree.odm_orthophoto_corners) as f:
                    for lineNumber, line in enumerate(f):
                        if lineNumber == 0:
                            tokens = line.split(' ')
                            if len(tokens) == 4:
                                ulx = float(tokens[0]) + \
                                    float(reconstruction.georef.utm_east_offset)
                                lry = float(tokens[1]) + \
                                    float(reconstruction.georef.utm_north_offset)
                                lrx = float(tokens[2]) + \
                                    float(reconstruction.georef.utm_east_offset)
                                uly = float(tokens[3]) + \
                                    float(reconstruction.georef.utm_north_offset)
                log.ODM_INFO('Creating GeoTIFF')

                orthophoto_vars = orthophoto.get_orthophoto_vars(args)

                kwargs = {
                    'ulx': ulx,
                    'uly': uly,
                    'lrx': lrx,
                    'lry': lry,
                    'vars': ' '.join(['-co %s=%s' % (k, orthophoto_vars[k]) for k in orthophoto_vars]),
                    'proj': reconstruction.georef.proj4(),
                    'png': tree.odm_orthophoto_file,
                    'tiff': tree.odm_orthophoto_tif,
                    'log': tree.odm_orthophoto_tif_log,
                    'max_memory': get_max_memory(),
                }

                system.run('gdal_translate -a_ullr {ulx} {uly} {lrx} {lry} '
                           '{vars} '
                           '-a_srs \"{proj}\" '
                           '--config GDAL_CACHEMAX {max_memory}% '
                           '{png} {tiff} > {log}'.format(**kwargs))

                bounds_file_path = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.gpkg')
                    
                # Cutline computation, before cropping
                # We want to use the full orthophoto, not the cropped one.
                if args.orthophoto_cutline:
                    compute_cutline(tree.odm_orthophoto_tif, 
                                    bounds_file_path,
                                    os.path.join(tree.odm_orthophoto, "cutline.gpkg"),
                                    args.max_concurrency,
                                    tmpdir=os.path.join(tree.odm_orthophoto, "grass_cutline_tmpdir"),
                                    scale=0.25)

                if args.crop > 0:
                    Cropper.crop(bounds_file_path, tree.odm_orthophoto_tif, orthophoto_vars)

                if args.build_overviews:
                    orthophoto.build_overviews(tree.odm_orthophoto_tif)

                geotiffcreated = True
            if not geotiffcreated:
                log.ODM_WARNING('No geo-referenced orthophoto created due '
                                'to missing geo-referencing or corner coordinates.')

        else:
            log.ODM_WARNING('Found a valid orthophoto in: %s' % tree.odm_orthophoto_file)
