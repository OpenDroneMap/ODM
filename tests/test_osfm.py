import unittest
import os
from opendm.osfm import get_submodel_argv, get_submodel_args_dict
from opendm import config
from unittest import mock

class TestOSFM(unittest.TestCase):
    def test_setup_preserves_seeded_reference_lla(self):
        context = __import__('opendm.osfm', fromlist=['OSFMContext']).OSFMContext('/tmp')
        reconstruction = mock.Mock()
        reconstruction.is_georeferenced.return_value = True
        reconstruction.georef.utm_east_offset = 1
        reconstruction.georef.utm_north_offset = 2
        reconstruction.georef.proj4.return_value = "+proj=utm"
        with mock.patch.object(context, "path", return_value="/tmp/reference_lla.json"), \
                mock.patch("opendm.osfm.io.file_exists", return_value=True), \
                mock.patch.object(context, "write_reference_lla") as write_reference:
            context.write_reference_lla_if_needed(reconstruction)
            write_reference.assert_not_called()

    def setUp(self):
        pass

    def test_get_submodel_argv(self):
        # Base
        args = config.config(["--project-path", "/datasets"])
        
        self.assertEqual(get_submodel_argv(args)[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report'])
        self.assertEqual(get_submodel_argv(args, "/submodels", "submodel_0000")[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report', '--project-path', '/submodels', 'submodel_0000'])        

        # Base + project name
        args = config.config(["--project-path", "/datasets", "brighton"])
        self.assertEqual(get_submodel_argv(args)[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report'])
        self.assertEqual(get_submodel_argv(args, "/submodels", "submodel_0000")[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report', '--project-path', '/submodels', 'submodel_0000'])        

        # Project name + base
        args = config.config(["brighton", "--project-path", "/datasets"])
        self.assertEqual(get_submodel_argv(args)[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report'])
        self.assertEqual(get_submodel_argv(args, "/submodels", "submodel_0000")[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report', '--project-path', '/submodels', 'submodel_0000'])        

        # Crop
        args = config.config(["brighton", "--project-path", "/datasets", "--crop", "0"])
        self.assertEqual(get_submodel_argv(args)[1:], 
            ['--crop', '0.015625', '--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report'])
        self.assertEqual(get_submodel_argv(args, "/submodels", "submodel_0000")[1:], 
            ['--crop', '0.015625', '--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report', '--project-path', '/submodels', 'submodel_0000'])        

        # With sm-cluster, pc-csv and others
        args = config.config(["--project-path", "/datasets", "--split", "200", "--pc-csv"])
        self.assertEqual(get_submodel_argv(args)[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report'])
        self.assertEqual(get_submodel_argv(args, "/submodels", "submodel_0000")[1:], 
            ['--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report', '--project-path', '/submodels', 'submodel_0000'])        

        # Cameras JSON
        args = config.config(["--project-path", "/datasets", "--cameras", os.path.join(os.path.dirname(os.path.realpath(__file__)), "assets", "sample.json")])
        self.assertEqual(get_submodel_argv(args)[1:], 
            ['--cameras', '{"test": "1"}', '--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report'])
        
        # Camera JSON string
        args = config.config(["--project-path", "/datasets", "--cameras", '{"test": "1"}'])
        self.assertEqual(get_submodel_argv(args)[1:], 
            ['--cameras', '{"test": "1"}', '--orthophoto-cutline', '--dem-euclidean-map', '--skip-3dmodel', '--skip-report'])
    
    def test_get_submodel_argv_dict(self):
        # Base
        args = config.config(["--project-path", "/datasets"])
        
        self.assertEqual(get_submodel_args_dict(args), 
            {'orthophoto-cutline': True, 'skip-3dmodel': True, 'dem-euclidean-map': True, 'skip-report': True})

if __name__ == '__main__':
    unittest.main()