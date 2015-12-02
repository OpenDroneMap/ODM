import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context

class ODMGeoreferencingCell(ecto.Cell):

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("texture_path", "The application arguments.", {})

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD Georeferencing Cell')

        # get inputs
        args = self.inputs.args
        project_path = io.absolute_path_file(args['project_path'])

        # define paths and create working directories
        odm_texturing = io.join_paths(project_path, 'odm_texturing')
        odm_georeferencing = io.join_paths(project_path, 'odm_georeferencing')
        system.mkdir_p(odm_georeferencing)

        images_path = io.join_paths(project_path, 'images_resize')
        images_list_file = io.join_paths(project_path, 'opensfm/list_r000.out')

        # define gcp file path, we'll assume that's placed in the project root
        gcp_file = io.join_paths(project_path, args['odm_georeferencing_gcpFile'])
        coords_file = io.join_paths(odm_georeferencing, 'coords.txt')

        # in case a gcp file it's not provided, let's try to generate it using
        # images metadata. Internally calls jhead.
        if not args['odm_georeferencing_useGcp'] and not io.file_exists(coords_file):
            log.ODM_WARNING('Warning: No coordinates file. ' \
                'Generating coordinates file in: %s' % coords_file)
            try:
                log_file = io.join_paths(odm_georeferencing, 'odm_texturing_utm_log.txt')

                system.run('%s/odm_extract_utm -imagesPath %s/ '      \
                    '-imageListFile %s -outputCoordFile %s '          \
                    '-logFile %s' %                                   \
                    (context.odm_modules_path, images_path,           \
                    images_list_file, coords_file, log_file))
            except Exception, e:
                log.ODM_ERROR('Could not generate GCP file from images metadata.' \
                    'Consider rerunning with argument --odm_georeferencing-useGcp'\
                     ' and provide a proper GCP file')
                log.ODM_ERROR(str(e))
                return ecto.QUIT
        elif io.file_exists(coords_file):
            log.ODM_WARNING('Found a valid coordinates file in: %s' % coords_file)

        # define odm georeferencing outputs
        # for convenience we'll put all data into odm_texturing
        model_geo = io.join_paths(odm_texturing, 'odm_textured_model_geo.obj')
        pointcloud_geo = io.join_paths(odm_texturing, 'odm_textured_model_geo.ply')
        system_geo = io.join_paths(odm_texturing, 'odm_textured_model_geo.txt')

        # check if we rerun cell or not
        rerun_cell = args['run_only'] is not None \
            and args['run_only'] == 'odm_georeferencing'

        if not io.file_exists(model_geo) or \
            not io.file_exists(pointcloud_geo) or rerun_cell:

            # odm_georeference definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'bundle': io.join_paths(project_path,'opensfm/bundle_r000.out'),
                'gcp': io.join_paths(project_path, args['odm_georeferencing_gcpFile']),
                'imgs': io.join_paths(project_path, 'images_resize'),
                'imgs_list': io.join_paths(project_path, 'opensfm/list_r000.out'),
                'size': str(args['resize_to']),
                'model': io.join_paths(project_path, 'odm_texturing/odm_textured_model.obj'),
                'pc': io.join_paths(project_path, 'pmvs/recon0/models/pmvs_options.txt.ply'),
                'log': io.join_paths(odm_georeferencing, 'odm_texturing_log.txt'),
                'coords': io.join_paths(odm_georeferencing, 'coords.txt'),
                'pc_geo': pointcloud_geo,
                'geo_sys': system_geo,
                'model_geo': model_geo,

            }

            # run odm_georeference
            system.run('{bin}/odm_georef -bundleFile {bundle} -inputCoordFile {coords} ' \
                '-bundleResizedTo {size} -inputFile {model} -outputFile {model_geo} '    \
                '-inputPointCloudFile {pc} -outputPointCloudFile {pc_geo} '              \
                '-logFile {log} -georefFileOutputPath {geo_sys}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid georeferenced model in: %s' % pointcloud_geo)


        log.ODM_INFO('Running OMD Georeferencing Cell - Finished')
        return ecto.OK if args['end_with'] != 'odm_georeferencing' else ecto.QUIT


def odm_georeferencing():
    """Run odm_georeferencing"""
    print "\n  - running georeferencing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_georeferencing")
    except:
        pass

    if not args.odm_georeferencing_useGcp:
        run("\"" + BIN_PATH + "/odm_extract_utm\" -imagesPath " + jobOptions["srcDir"] + "/ -imageListFile " \
            + jobOptions["jobDir"] + "/pmvs/list.rd.txt -outputCoordFile " + jobOptions["jobDir"]            \
            + "/odm_georeferencing/coordFile.txt")
        
        run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + jobOptions["jobDir"]                            \
            + "/pmvs/bundle.rd.out -inputCoordFile " + jobOptions["jobDir"]                                  \
            + "/odm_georeferencing/coordFile.txt -inputFile " + jobOptions["jobDir"]                         \
            + "-results/odm_texturing/odm_textured_model.obj -outputFile " + jobOptions["jobDir"]            \
            + "-results/odm_texturing/odm_textured_model_geo.obj -inputPointCloudFile "                      \
            + jobOptions["jobDir"] + "-results/option-0000.ply -outputPointCloudFile " + jobOptions["jobDir"] \
            + "-results/option-0000_georef.ply -logFile " + jobOptions["jobDir"]                             \
            + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + jobOptions["jobDir"] \
            + "-results/odm_texturing/odm_textured_model_geo_georef_system.txt")
        
    elif os.path.isfile(jobOptions["srcDir"] + "/" + args.odm_georeferencing_gcpFile):
        run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + jobOptions["jobDir"]                             \
            + "/pmvs/bundle.rd.out -gcpFile " + jobOptions["srcDir"] + "/" + args.odm_georeferencing_gcpFile  \
            + " -imagesPath " + jobOptions["srcDir"] + "/ -imagesListPath " + jobOptions["jobDir"]            \
            + "/pmvs/list.rd.txt -bundleResizedTo " + str(jobOptions["resizeTo"]) + " -inputFile "            \
            + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model.obj -outputFile "             \
            + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo.obj -outputCoordFile "    \
            + jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt -inputPointCloudFile "                \
            + jobOptions["jobDir"] + "-results/option-0000.ply -outputPointCloudFile " + jobOptions["jobDir"] \
            + "-results/option-0000_georef.ply -logFile " + jobOptions["jobDir"]                              \
            + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + jobOptions["jobDir"]  \
            + "-results/odm_texturing/odm_textured_model_geo_georef_system.txt")
    else:
        print "Warning: No GCP file. Consider rerunning with argument --odm_georeferencing-useGcp false --start-with odm_georeferencing"
        print "Skipping orthophoto"
        args.end_with = "odm_georeferencing"

    if "csString" not in jobOptions:
        parse_coordinate_system()

    if "csString" in jobOptions and "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions:
        images = []
        with open(jobOptions["jobDir"] + "/pmvs/list.rd.txt") as f:
            images = f.readlines()

        if len(images) > 0:
            with open(jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt") as f:
                for lineNumber, line in enumerate(f):
                    if lineNumber >= 2 and lineNumber - 2 < len(images):
                        tokens = line.split(' ')
                        
                        if len(tokens) >= 3:
                            x = float(tokens[0])
                            y = float(tokens[1])
                            z = float(tokens[2])
                            filename = images[lineNumber - 2]
                            
                            run("echo " + str(x + jobOptions["utmEastOffset"]) + " "                          \
                                + str(y + jobOptions["utmNorthOffset"]) + " " + str(z)                        \
                                + " | cs2cs " + jobOptions["csString"] + " +to +datum=WGS84 +proj=latlong > " \
                                + jobOptions["jobDir"] + "/odm_georeferencing/latlong.txt")
                            
                            with open(jobOptions["jobDir"] + "/odm_georeferencing/latlong.txt") as latlongFile:
                                latlongLine = latlongFile.readline()
                                tokens = latlongLine.split()
                                if len(tokens) >= 2:
                                    exifGpsInfoWritten = False

                                    lonString = tokens[0]  # Example: 83d18'16.285"W
                                    latString = tokens[1]  # Example: 41d2'11.789"N
                                    altString = ""
                                    if len(tokens) > 2:
 
                                        altString = tokens[2] # Example: 0.998

                                    tokens = re.split("[d '\"]+", lonString)
                                    if len(tokens) >= 4:
                                        lonDeg = tokens[0]
                                        lonMin = tokens[1]
                                        lonSec = tokens[2]
                                        lonSecFrac = fractions.Fraction(lonSec)
                                        lonSecNumerator = str(lonSecFrac._numerator)
                                        lonSecDenominator = str(lonSecFrac._denominator)
                                        lonRef = tokens[3]

                                        tokens = re.split("[d '\"]+", latString)
                                        if len(tokens) >= 4:
                                            latDeg = tokens[0]
                                            latMin = tokens[1]
                                            latSec = tokens[2]
                                            latSecFrac = fractions.Fraction(latSec)
                                            latSecNumerator = str(latSecFrac._numerator)
                                            latSecDenominator = str(latSecFrac._denominator)
                                            latRef = tokens[3]

                                            exivCmd = "exiv2 -q"
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitude " + latDeg + "/1 "         \
                                                + latMin + "/1 " + latSecNumerator + "/" + latSecDenominator + "\""
                                            
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitudeRef " + latRef + "\""
                                            
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitude " + lonDeg + "/1 "         \
                                                + lonMin + "/1 " + lonSecNumerator + "/" + lonSecDenominator + "\""
                                            
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitudeRef " + lonRef + "\""

                                            altNumerator = arcDenominator = 0  # BUG: arcDenominator is never used
                                            
                                            if altString:
                                                altFrac = fractions.Fraction(altString)
                                                altNumerator = str(altFrac._numerator)
                                                altDenominator = str(altFrac._denominator)
                                                exivCmd += " -M\"set Exif.GPSInfo.GPSAltitude " + altNumerator + "/" + altDenominator + "\""
                                                exivCmd += " -M\"set Exif.GPSInfo.GPSAltitudeRef 0\""

                                            exivCmd += " " + filename
                                            run(exivCmd)
                                            exifGpsInfoWritten = True

                                    if not exifGpsInfoWritten:
                                        print("    Warning: Failed setting EXIF GPS info for " \
                                              + filename + " based on " + latlongLine)

    if "epsg" in jobOptions and "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions:
        lasCmd = "\"" + BIN_PATH + "/txt2las\" -i " + jobOptions["jobDir"] +                                         \
            "-results/option-0000_georef.ply -o " + jobOptions["jobDir"]                                             \
            + "-results/pointcloud_georef.laz -skip 30 -parse xyzRGBssss -set_scale 0.01 0.01 0.01 -set_offset "     \
            + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"]) + " 0 -translate_xyz "      \
            + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"])                             \
            + " 0 -epsg " + str(jobOptions["epsg"])
        
        print("    Creating geo-referenced LAS file (expecting warning)...")
        
        run(lasCmd)
 

    if args['--end-with'] != "odm_georeferencing":
        odm_orthophoto()