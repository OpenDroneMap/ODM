import os
import sys
import multiprocessing

# Define some needed locations
current_path = os.path.abspath(os.path.dirname(__file__))
root_path, _ = os.path.split(current_path)

superbuild_path = os.path.join(root_path, 'SuperBuild')
superbuild_bin_path = os.path.join(superbuild_path, 'install', 'bin')
tests_path = os.path.join(root_path, 'tests')
tests_data_path = os.path.join(root_path, 'tests/test_data')

# add opencv,opensfm to python path
python_packages_paths = [os.path.join(superbuild_path, p) for p in [
    'install/lib/python3.8/dist-packages',
    'install/lib/python3/dist-packages',
    'src/opensfm'
]]
for p in python_packages_paths:
    sys.path.append(p)


# define opensfm path
opensfm_path = os.path.join(superbuild_path, "src/opensfm")

# define orb_slam2 path
orb_slam2_path = os.path.join(superbuild_path, "src/orb_slam2")

poisson_recon_path = os.path.join(superbuild_path, 'src', 'PoissonRecon', 'Bin', 'Linux', 'PoissonRecon')
dem2mesh_path = os.path.join(superbuild_path, 'src', 'dem2mesh', 'dem2mesh')
dem2points_path = os.path.join(superbuild_path, 'src', 'dem2points', 'dem2points')

# define mvstex path
mvstex_path = os.path.join(superbuild_path, "install/bin/texrecon")

# openmvs paths
omvs_densify_path = os.path.join(superbuild_path, "install/bin/OpenMVS/DensifyPointCloud")

# define txt2las path
txt2las_path = os.path.join(superbuild_path, 'src/las-tools/bin')
pdal_path = os.path.join(superbuild_path, 'build/pdal/bin')

# define odm modules path
odm_modules_path = os.path.join(root_path, "build/bin")
odm_modules_src_path = os.path.join(root_path, "modules")

settings_path = os.path.join(root_path, 'settings.yaml')

# Define supported image extensions
supported_extensions = {'.jpg','.jpeg','.png', '.tif', '.tiff', '.bmp'}

# Define the number of cores
num_cores = multiprocessing.cpu_count()


# Print python paths if invoked as a script
if __name__ == "__main__":
    print("export PYTHONPATH=" + ":".join(python_packages_paths))