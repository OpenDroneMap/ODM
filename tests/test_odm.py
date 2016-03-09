import unittest
import os

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


def setUp():
    # Run tests
    print '%s' % options
    options.project_path = context.tests_data_path
    # options.rerun_all = True
    app, plasm = appSetup(options)
    print 'Run Setup: Initial Run'
    run_plasm(options, plasm)
    # options.rerun_all = False


class TestResize(unittest.TestCase):
    """
    Tests the resize function
    """

    def test_resize(self):
        # rerun resize cell and set params
        options.rerun = 'resize'
        options.resize_to = 1600
        # rebuild app
        self.app, self.plasm = appSetup(options)
        run_plasm(options, self.plasm)
        # assert each image is sized to the option.resize_to
        print "Run Test 1: Check resize 1600"
        self.assertEquals(max(self.app.resize.outputs.photos[0].height, self.app.resize.outputs.photos[0].width),
                          options.resize_to)

    def test_all_resized(self):
        options.rerun = 'resize'
        options.resize_to = 1600
        print "Run Test 2: Rerun resize: %s" % options.resize_to
        # rebuild app
        self.app, self.plasm = appSetup(options)
        run_plasm(options, self.plasm)
        # assert the number of images in images == number of images in resize
        print "Run Test 1: Check that the resize happens"
        self.assertEquals(len(self.app.resize.outputs.photos), len(self.app.dataset.outputs.photos))


# class TestOpenSfM(unittest.TestCase):
#     """
#     Tests the OpenSfM module
#     """
#
#     def test_opensfm(self):
#         options.rerun = 'opensfm'
#         self.app, self.plasm = appSetup(options)
#         run_plasm(options, self.plasm)
#         # Test configuration
#         self.assertEquals(self.app.opensfm)

class TestPMVS(unittest.TestCase):

    def test_pmvs(self):
        self.assertTrue(os.path.isfile(context.pmvs2_path))


if __name__ == '__main__':
    unittest.main()
