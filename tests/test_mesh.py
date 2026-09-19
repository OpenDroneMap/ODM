import os
import tempfile
import unittest
from unittest.mock import patch

from opendm import mesh


class TestMesh(unittest.TestCase):
    def test_poisson_uses_output_directory_for_temporary_files(self):
        with tempfile.TemporaryDirectory() as root:
            mesh_dir = os.path.join(root, "mesh output")
            os.mkdir(mesh_dir)
            point_cloud = os.path.join(root, "cloud.ply")
            output = os.path.join(mesh_dir, "odm_mesh.ply")
            dirty = os.path.join(mesh_dir, "odm_mesh.dirty.ply")
            commands = []

            def run(command):
                commands.append(command)
                if "--linearFit" in command:
                    with open(dirty, "wb"):
                        pass

            with patch.object(mesh.system, "run", side_effect=run):
                with patch.object(
                    mesh.context, "poisson_recon_path", "PoissonRecon"
                ):
                    with patch.object(
                        mesh.context,
                        "omvs_reconstructmesh_path",
                        "ReconstructMesh",
                    ):
                        result = mesh.screened_poisson_reconstruction(
                            point_cloud, output, threads=3
                        )

            self.assertEqual(result, output)
            self.assertEqual(len(commands), 2)
            self.assertIn(
                '--tempDir "{}"'.format(os.path.abspath(mesh_dir)),
                commands[0],
            )


if __name__ == "__main__":
    unittest.main()
