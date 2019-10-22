import os
import sys
from opendm import io
import multiprocessing

# Define some needed locations
current_path = os.path.abspath(os.path.dirname(__file__))
root_path, _ = os.path.split(current_path)

superbuild_path = os.path.join(root_path, 'SuperBuild')
superbuild_bin_path = os.path.join(superbuild_path, 'install', 'bin')
tests_path = os.path.join(root_path, 'tests')
tests_data_path = os.path.join(root_path, 'tests/test_data')

# add opencv to python path
pyopencv_path = os.path.join(superbuild_path, 'install/lib/python2.7/dist-packages')
sys.path.append(pyopencv_path)

# define opensfm path
opensfm_path = os.path.join(superbuild_path, "src/opensfm")

# define orb_slam2 path
orb_slam2_path = os.path.join(superbuild_path, "src/orb_slam2")

# define mve join_paths
makescene_path = os.path.join(superbuild_path, 'src', 'elibs', 'mve', 'apps', 'makescene', 'makescene') #TODO: don't install in source
dmrecon_path = os.path.join(superbuild_path, 'src', 'elibs', 'mve', 'apps', 'dmrecon', 'dmrecon')
scene2pset_path = os.path.join(superbuild_path, 'src', 'elibs', 'mve', 'apps', 'scene2pset', 'scene2pset')
meshclean_path = os.path.join(superbuild_path, 'src', 'elibs', 'mve', 'apps', 'meshclean', 'meshclean')

poisson_recon_path = os.path.join(superbuild_path, 'src', 'PoissonRecon', 'Bin', 'Linux', 'PoissonRecon')
dem2mesh_path = os.path.join(superbuild_path, 'src', 'dem2mesh', 'dem2mesh')
dem2points_path = os.path.join(superbuild_path, 'src', 'dem2points', 'dem2points')

# define mvstex path
mvstex_path = os.path.join(superbuild_path, "install/bin/texrecon")

# define txt2las path
txt2las_path = os.path.join(superbuild_path, 'src/las-tools/bin')
pdal_path = os.path.join(superbuild_path, 'build/pdal/bin')

# define odm modules path
odm_modules_path = os.path.join(root_path, "build/bin")
odm_modules_src_path = os.path.join(root_path, "modules")

settings_path = os.path.join(root_path, 'settings.yaml')

# Define supported image extensions
supported_extensions = {'.jpg','.jpeg','.png', '.tif', '.tiff'}

# Define the number of cores
num_cores = multiprocessing.cpu_count()
