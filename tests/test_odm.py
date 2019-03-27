import unittest
import os
import shutil

import ecto
from opendm import config
from opendm import context
from scripts.odm_app import ODMApp
from ecto.opts import scheduler_options, run_plasm

parser = config.parser
scheduler_options(parser)
options = config.config()


def appSetup(options):
    app = ODMApp(args=options)
    plasm = ecto.Plasm()
    plasm.insert(app)
    return app, plasm


def setup_module():
    # Run tests
    print '%s' % options
    options.project_path = context.tests_data_path
    # options.rerun_all = True
    app, plasm = appSetup(options)
    print 'Run Setup: Initial Run'
    run_plasm(options, plasm)
    # options.rerun_all = False


def teardown_module():
    # Delete generated test directories
    dirnames = ['opensfm', 'odm_meshing',
                'odm_texturing', 'odm_georeferencing', 'odm_orthophoto']
    for n in dirnames:
        rmpath = os.path.join(context.tests_data_path, n)
        if os.path.exists(rmpath):
            shutil.rmtree(rmpath)


class TestOpenSfM(unittest.TestCase):
    """
    Tests the OpenSfM module
    """
    def setUp(self):
        options.rerun = 'opensfm'
        self.app, self.plasm = appSetup(options)
        run_plasm(options, self.plasm)

    def test_opensfm(self):
        # Test configuration
        self.assertTrue(os.path.isfile(self.app.opensfm.inputs.tree.opensfm_reconstruction))


class TestMeshing(unittest.TestCase):

    def setUp(self):
        options.rerun = 'odm_meshing'
        self.app, self.plasm = appSetup(options)
        run_plasm(options, self.plasm)

    def test_meshing(self):
        self.assertTrue(os.path.isfile(self.app.meshing.inputs.tree.odm_mesh))


class TestTexturing(unittest.TestCase):

    def setUp(self):
        options.rerun = 'odm_texturing'
        self.app, self.plasm = appSetup(options)
        run_plasm(options, self.plasm)

    def test_texturing(self):
        self.assertTrue(os.path.isfile(self.app.texturing.inputs.tree.odm_textured_model_obj))


class TestGeoreferencing(unittest.TestCase):

    def setUp(self):
        options.rerun = 'odm_georeferencing'
        self.app, self.plasm = appSetup(options)
        run_plasm(options, self.plasm)

    def test_georef(self):
        self.assertTrue(os.path.isfile(self.app.georeferencing.inputs.tree.odm_georeferencing_coords) &
                        os.path.isfile(self.app.georeferencing.inputs.tree.odm_georeferencing_model_obj_geo))

    def test_las_out(self):
        self.assertTrue(os.path.isfile(os.path.join(self.app.georeferencing.inputs.tree.odm_georeferencing,
                                                    "odm_georeferenced_model.laz")))


class TestOrthophoto(unittest.TestCase):

    def setUp(self):
        options.rerun = 'odm_orthophoto'
        self.app, self.plasm = appSetup(options)
        run_plasm(options, self.plasm)

    def test_orthophoto(self):
        self.assertTrue(os.path.isfile(self.app.orthophoto.inputs.tree.odm_orthophoto_file))


if __name__ == '__main__':
    unittest.main()