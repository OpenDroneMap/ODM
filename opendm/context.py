import os
import sys
import multiprocessing

# Define some needed locations
current_path = os.path.abspath(os.path.dirname(__file__))
root_path, _ = os.path.split(current_path)

superbuild_path = os.path.join(root_path, 'SuperBuild')
superbuild_bin_path = os.path.join(superbuild_path, 'install', 'bin')

# add opencv,opensfm to python path
python_packages_paths = [os.path.join(superbuild_path, p) for p in [
    'install/lib/python3.9/dist-packages',
    'install/lib/python3.8/dist-packages',
    'install/lib/python3/dist-packages',
    'install/bin/opensfm'
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
supported_extensions = {'.jpg','.jpeg','.png', '.tif', '.tiff', '.bmp'}

# Define the number of cores
num_cores = multiprocessing.cpu_count()


# Print python paths if invoked as a script
if __name__ == "__main__":
    print("export PYTHONPATH=" + ":".join(python_packages_paths))