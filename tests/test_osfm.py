import json
import os
import tempfile
import unittest
import zipfile
from types import SimpleNamespace

from opendm.osfm import OSFMContext, get_submodel_argv, get_submodel_args_dict
from opendm.remote import ReconstructionTask
from opendm import config


class TestOSFM(unittest.TestCase):
    def make_reconstruction(self):
        photo = SimpleNamespace(filename="synthetic.jpg", altitude=100,
                                latitude=0, longitude=3, mask=None,
                                width=4000, height=3000)
        georef = SimpleNamespace(utm_east_offset=500000, utm_north_offset=0,
                                 proj4=lambda: "+proj=utm +zone=31 +datum=WGS84 +units=m +no_defs")
        return SimpleNamespace(photos=[photo], multi_camera=None, georef=georef,
                               gcp=SimpleNamespace(gcp_path=None),
                               is_georeferenced=lambda: True)

    def test_setup_preserves_seeded_reference_lla(self):
        with tempfile.TemporaryDirectory() as root:
            parent = os.path.join(root, "parent")
            remote = os.path.join(root, "remote")
            os.makedirs(os.path.join(parent, "opensfm"))
            os.makedirs(remote)
            parent_reference = {"latitude": 0.001, "longitude": 3.001, "altitude": 0.0}
            with open(os.path.join(parent, "opensfm", "reference_lla.json"), "w") as f:
                json.dump(parent_reference, f)

            task = ReconstructionTask(parent, None, {})
            seed = task.create_seed_payload(["opensfm/reference_lla.json"])
            with zipfile.ZipFile(seed) as archive:
                self.assertEqual(archive.namelist(), ["opensfm/reference_lla.json"])
                archive.extractall(remote)

            args = config.config(["--project-path", remote])
            context = OSFMContext(os.path.join(remote, "opensfm"))
            context.setup(args, os.path.join(remote, "images"), self.make_reconstruction())

            with open(context.path("reference_lla.json")) as f:
                self.assertEqual(json.load(f), parent_reference)
            self.assertTrue(os.path.isfile(context.path("image_list.txt")))

    def test_setup_writes_reference_lla_for_fresh_and_rerun_projects(self):
        with tempfile.TemporaryDirectory() as root:
            args = config.config(["--project-path", root])
            context = OSFMContext(os.path.join(root, "opensfm"))
            reconstruction = self.make_reconstruction()
            for rerun in (False, True):
                if rerun:
                    with open(context.path("reference_lla.json"), "w") as f:
                        json.dump({"latitude": 0.001, "longitude": 3.001, "altitude": 0.0}, f)
                context.setup(args, os.path.join(root, "images"), reconstruction, rerun=rerun)
                with open(context.path("reference_lla.json")) as f:
                    result = json.load(f)
                self.assertAlmostEqual(result["latitude"], 0.0, places=7)
                self.assertAlmostEqual(result["longitude"], 3.0, places=7)
                self.assertEqual(result["altitude"], 0.0)

    def test_submodel_arguments_remove_rerun_options(self):
        for option in (("--rerun-all",), ("--rerun", "opensfm"),
                       ("--rerun-from", "opensfm")):
            with self.subTest(option=option):
                args = config.config(["--project-path", "/datasets", *option])
                submodel_args = get_submodel_args_dict(args)
                self.assertNotIn("rerun", submodel_args)
                self.assertNotIn("rerun-from", submodel_args)
                self.assertNotIn("rerun-all", submodel_args)

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