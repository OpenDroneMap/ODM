import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context

class ODMOrthoPhotoCell(ecto.Cell):

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD OrthoPhoto Cell')

        # get inputs
        args = self.inputs.args
        project_path = io.absolute_path_file(args['project_path'])

        # define paths and create working directories
        odm_texturing = io.join_paths(project_path, 'odm_texturing')
        odm_orthophoto = io.join_paths(project_path, 'odm_orthophoto')
        system.mkdir_p(odm_orthophoto)

        # odm_georeference definitions
        kwargs = {
            'bin': context.odm_modules_path,
            'model_geo': io.join_paths(odm_texturing, 'odm_textured_model_geo.obj'),
            'log': io.join_paths(odm_orthophoto, 'odm_orthophoto_log.txt'),
            'ortho': io.join_paths(odm_orthophoto, 'odm_orthphoto.png'),
            'corners': io.join_paths(odm_orthophoto, 'odm_orthphoto_corners.txt'),
            'res': str(20.0)
        }

        # check if we rerun cell or not
        rerun_cell = args['run_only'] is not None \
            and args['run_only'] == 'odm_orthophoto'

        if not io.file_exists(kwargs['ortho']) or rerun_cell:
            # run odm_georeference
            system.run('{bin}/odm_orthophoto -inputFile {model_geo} -logFile {log} ' \
            '-outputFile {ortho} -resolution {res} -outputCornerFile {corners}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid orthophoto in: %s' % kwargs['ortho'])

        log.ODM_INFO('Running OMD OrthoPhoto Cell - Finished')
        return ecto.OK if args['end_with'] != 'odm_orthophoto' else ecto.QUIT


def odm_orthophoto():
    """Run odm_orthophoto"""
    print "\n  - running orthophoto generation - " + system.now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_orthophoto")
    except:
        pass

    run("\"" + BIN_PATH + "/odm_orthophoto\" -inputFile " + jobOptions["jobDir"] +                \
        "-results/odm_texturing/odm_textured_model_geo.obj -logFile " + jobOptions["jobDir"]      \
        + "/odm_orthophoto/odm_orthophoto_log.txt -outputFile " + jobOptions["jobDir"]            \
        + "-results/odm_orthphoto.png -resolution 20.0 -outputCornerFile " + jobOptions["jobDir"] \
        + "/odm_orthphoto_corners.txt")

    if "csString" not in jobOptions:
        parse_coordinate_system()

    geoTiffCreated = False
    if ("csString" in jobOptions and
            "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions):
        ulx = uly = lrx = lry = 0.0
        with open(jobOptions["jobDir"] +
                  "/odm_orthphoto_corners.txt") as f:
            for lineNumber, line in enumerate(f):
                if lineNumber == 0:
                    tokens = line.split(' ')
                    if len(tokens) == 4:
                        ulx = float(tokens[0]) + \
                            float(jobOptions["utmEastOffset"])
                        lry = float(tokens[1]) + \
                            float(jobOptions["utmNorthOffset"])
                        lrx = float(tokens[2]) + \
                            float(jobOptions["utmEastOffset"])
                        uly = float(tokens[3]) + \
                            float(jobOptions["utmNorthOffset"])

        print("    Creating GeoTIFF...")
        sys.stdout.write("    ")
        run("gdal_translate -a_ullr " + str(ulx) + " " + str(uly) + " " +
            str(lrx) + " " + str(lry) + " -a_srs \"" + jobOptions["csString"] +
            "\" " + jobOptions["jobDir"] + "-results/odm_orthphoto.png " +
            jobOptions["jobDir"] + "-results/odm_orthphoto.tif")
        geoTiffCreated = True

    if not geoTiffCreated:
 
        print "    Warning: No geo-referenced orthophoto created due to missing geo-referencing or corner coordinates."