import ecto

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
        params.declare("use_gcp", 'set to true for enabling GCPs from the file above', False)
        params.declare("img_size", 'image size used in calibration', 2400)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("photos", "list of ODMPhoto's", [])
        inputs.declare("reconstruction", "list of ODMReconstructions", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD Georeferencing Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree

        # define paths and create working directories
        system.mkdir_p(tree.odm_georeferencing)

        # in case a gcp file it's not provided, let's try to generate it using
        # images metadata. Internally calls jhead.
        if not self.params.use_gcp and \
           not io.file_exists(tree.odm_georeferencing_coords):
            
            log.ODM_WARNING('Warning: No coordinates file. ' \
                'Generating coordinates file in: %s' % tree.odm_georeferencing_coords)
            try:
                # odm_georeference definitions
                kwargs = {
                    'bin': context.odm_modules_path,
                    'imgs': tree.dataset_resize,
                    'imgs_list': tree.opensfm_bundle_list,
                    'coords': tree.odm_georeferencing_coords,
                    'log': tree.odm_georeferencing_utm_log
                }

                # run UTM extraction binary
                system.run('{bin}/odm_extract_utm -imagesPath {imgs}/ '      \
                    '-imageListFile {imgs_list} -outputCoordFile {coords} '  \
                    '-logFile {log}'.format(**kwargs))

            except Exception, e:
                log.ODM_ERROR('Could not generate GCP file from images metadata.'  \
                    'Consider rerunning with argument --odm_georeferencing-useGcp' \
                    ' and provide a proper GCP file')
                log.ODM_ERROR(e)
                return ecto.QUIT

        # check if we rerun cell or not
        rerun_cell = args['rerun'] is not None \
            and args['rerun'] == 'odm_georeferencing'

        if not io.file_exists(tree.odm_textured_model_obj_geo) or \
           not io.file_exists(tree.odm_textured_model_ply_geo) or rerun_cell:

            # odm_georeference definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'bundle': tree.opensfm_bundle,
                'imgs': tree.dataset_resize,
                'imgs_list': tree.opensfm_bundle_list,
                'model': tree.odm_textured_model_obj,
                'pc': tree.pmvs_model,
                'log': tree.odm_georeferencing_log,
                'coords': tree.odm_georeferencing_coords,
                'pc_geo': tree.odm_textured_model_ply_geo,
                'geo_sys': tree.odm_textured_model_txt_geo,
                'model_geo': tree.odm_textured_model_obj_geo,
                'size': self.params.img_size,
                'gcp': io.join_paths(tree.root_path, self.params.gcp_file),

            }

            if self.params.use_gcp and \
               io.file_exists(tree.odm_georeferencing_coords):

                system.run('{bin}/odm_georef -bundleFile {bundle} -inputCoordFile {coords} ' \
                    '-bundleResizedTo {size} -inputFile {model} -outputFile {model_geo} '    \
                    '-inputPointCloudFile {pc} -outputPointCloudFile {pc_geo} '              \
                    '-logFile {log} -georefFileOutputPath {geo_sys} -gcpFile {gcp} '         \
                    '-outputCoordFile {coords}'.format(**kwargs))
            else:
                system.run('{bin}/odm_georef -bundleFile {bundle} -inputCoordFile {coords} ' \
                    '-inputFile {model} -outputFile {model_geo} '                            \
                    '-inputPointCloudFile {pc} -outputPointCloudFile {pc_geo} '              \
                    '-logFile {log} -georefFileOutputPath {geo_sys}'.format(**kwargs))

        else:
            log.ODM_WARNING('Found a valid georeferenced model in: %s' \
                % tree.odm_textured_model_ply_geo)


        # update images metadata
        geo_ref = types.ODM_GeoRef()
        geo_ref.parse_coordinate_system(tree.odm_georeferencing_coords)

        for idx, photo in enumerate(self.inputs.photos):
            geo_ref.utm_to_latlon(tree.odm_georeferencing_latlon, photo, idx)

        # convert ply model to LAS reference system
        geo_ref.convert_to_las(tree.odm_textured_model_ply_geo, tree.odm_georeferencing_pdal)


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
