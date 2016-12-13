import os
import sys
import multiprocessing

# Define some needed locations
scripts_path = os.path.abspath(os.path.dirname(__file__))
root_path, _ = os.path.split(scripts_path)

superbuild_path = os.path.join(root_path, 'SuperBuild')
tests_path = os.path.join(root_path, 'tests')
tests_data_path = os.path.join(root_path, 'tests/test_data')

# add opencv to python path
pyopencv_path = os.path.join(superbuild_path, 'install/lib/python2.7/dist-packages')
sys.path.append(pyopencv_path)

# define opensfm path
opensfm_path = os.path.join(superbuild_path, "src/opensfm")
ccd_widths_path = os.path.join(opensfm_path, 'opensfm/data/sensor_data.json')

# define orb_slam2 path
orb_slam2_path = os.path.join(superbuild_path, "src/orb_slam2")

# define pmvs path
cmvs_path = os.path.join(superbuild_path, "install/bin/cmvs")
cmvs_opts_path = os.path.join(superbuild_path, "install/bin/genOption")
pmvs2_path = os.path.join(superbuild_path, "install/bin/pmvs2")

# define mvstex path
mvstex_path = os.path.join(superbuild_path, "install/bin/texrecon")

# define txt2las path
txt2las_path = os.path.join(superbuild_path, 'src/las-tools/bin')
pdal_path = os.path.join(superbuild_path, 'build/pdal/bin')

# define odm modules path
odm_modules_path = os.path.join(root_path, "build/bin")
odm_modules_src_path = os.path.join(root_path, "modules")

# Define supported image extensions
supported_extensions = {'.jpg','.jpeg','.png'}

# Define the number of cores 
num_cores = multiprocessing.cpu_count()
