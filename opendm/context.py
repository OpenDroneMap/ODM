import os
import sys
import multiprocessing

if sys.platform == "win32":
    def _add_windows_dll_dir(path):
        if path and os.path.isdir(path):
            os.add_dll_directory(os.path.abspath(path))

    _add_windows_dll_dir(os.path.join(os.path.dirname(__file__), "..", "SuperBuild", "install", "bin"))
    _conda = os.environ.get("CONDA_PREFIX")
    if _conda:
        for _sub in ("Library/bin", "bin", ""):
            _add_windows_dll_dir(os.path.join(_conda, _sub) if _sub else _conda)

# Define some needed locations
current_path = os.path.abspath(os.path.dirname(__file__))
root_path, _ = os.path.split(current_path)

superbuild_path = os.path.join(root_path, 'SuperBuild')
superbuild_bin_path = os.path.join(superbuild_path, 'install', 'bin')

# add OpenSfM to python path (avoid overriding conda-provided python packages)
python_packages_paths = [os.path.join(superbuild_path, p) for p in [
    'install/bin/opensfm',
]]
for p in python_packages_paths:
    sys.path.append(p)


# define opensfm path
opensfm_path = os.path.join(superbuild_bin_path, "opensfm")

poisson_recon_path = os.path.join(superbuild_bin_path, 'PoissonRecon')
dem2mesh_path = os.path.join(superbuild_bin_path, 'dem2mesh')
dem2points_path = os.path.join(superbuild_bin_path, 'dem2points')

# define mvstex path
mvstex_path = os.path.join(superbuild_bin_path, "texrecon")

# openmvs paths
omvs_densify_path = os.path.join(superbuild_bin_path, "OpenMVS", "DensifyPointCloud")
omvs_reconstructmesh_path = os.path.join(superbuild_bin_path, "OpenMVS", "ReconstructMesh")

fpcfilter_path = os.path.join(superbuild_bin_path, "FPCFilter")

odm_orthophoto_path = os.path.join(superbuild_bin_path, "odm_orthophoto")
settings_path = os.path.join(root_path, 'settings.yaml')

# Define supported image extensions
supported_extensions = {'.jpg','.jpeg','.png', '.tif', '.tiff', '.bmp', '.raw', '.dng', '.nef', '.jxl'}
supported_video_extensions = {'.mp4', '.mov', '.lrv', '.ts'}

# Define the number of cores
num_cores = multiprocessing.cpu_count()


# Print python paths if invoked as a script
if __name__ == "__main__":
    print("export PYTHONPATH=" + os.pathsep.join(python_packages_paths))
