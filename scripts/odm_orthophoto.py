import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context

class ODMOrthoPhotoCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("resolution", 'Orthophoto ground resolution in pixels/meter', 20)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD OrthoPhoto Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree

        # define paths and create working directories
        system.mkdir_p(tree.odm_orthophoto)

        # check if we rerun cell or not
        rerun_cell = args['rerun'] is not None \
            and args['rerun'] == 'odm_orthophoto'

        if not io.file_exists(tree.odm_orthophoto_file) or rerun_cell:

            # odm_georeference definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'model_geo': tree.odm_textured_model_obj_geo,
                'log': tree.odm_orthophoto_log,
                'ortho': tree.odm_orthophoto_file,
                'corners': tree.odm_orthophoto_corners,
                'res': self.params.resolution
            }

            # run odm_georeference
            system.run('{bin}/odm_orthophoto -inputFile {model_geo} -logFile {log} ' \
            '-outputFile {ortho} -resolution {res} -outputCornerFile {corners}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid orthophoto in: %s' % tree.odm_orthophoto_file)

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