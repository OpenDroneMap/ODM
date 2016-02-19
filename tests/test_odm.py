import unittest
from opendm import config
from opendm import context
import ecto
from scripts.odm_app import ODMApp
from ecto.opts import scheduler_options, run_plasm
#

class TestResize(unittest.TestCase):
    '''
    Tests the resize function
    '''
    def setUp(self):
        # Run tests
        self.parser = config.parser
        scheduler_options(self.parser)
        self.options = self.parser.parse_args()
        self.options.project_path = context.tests_data_path
        self.app, self.plasm = self.appSetup(self.options)
        print 'Run Setup: Initial Run'
        run_plasm(self.options, self.plasm)

    def test_resize(self):
        # rerun resize cell and set params
        self.options.rerun = 'resize'
        self.options.resize_to = 1600
        print "Run Test 1: Rerun resize: %s" % self.options.resize_to
        # rebuild app
        self.app, self.plasm = self.appSetup(self.options)
        run_plasm(self.options, self.plasm)
        # assert each image is sized to the option.resize_to
        print "Run Test 1: Check that the resize happens"
        if self.app.resize.outputs.photos[0].height > self.app.resize.outputs.photos[0].width:
            self.assertEquals(self.app.resize.outputs.photos[0].height, self.options.resize_to)
        else:
            self.assertEquals(self.app.resize.outputs.photos[0].width, self.options.resize_to)


    def appSetup(self, options):
        app = ODMApp(args=vars(options))
        plasm = ecto.Plasm()
        plasm.insert(app)
        return app, plasm

if __name__ == '__main__':
    unittest.main()
