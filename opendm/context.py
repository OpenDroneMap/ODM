import os
import sys
import multiprocessing

# Define some needed locations
scripts_path = os.path.abspath(os.path.dirname(__file__))
root_path, _ = os.path.split(scripts_path)

superbuild_path = os.path.join(root_path, 'SuperBuild')
ccd_widths_path = os.path.join(root_path, 'data/ccd_defs.json')

# add opencv to python path
pyopencv_path = os.path.join(superbuild_path, 'install/lib/python2.7/dist-packages')
sys.path.append(pyopencv_path)

# define opensfm path
opensfm_path = os.path.join(superbuild_path, "src/opensfm")

# define pmvs path
cmvs_path = os.path.join(superbuild_path, "install/bin/cmvs")
cmvs_opts_path = os.path.join(superbuild_path, "install/bin/genOption")
pmvs2_path = os.path.join(superbuild_path, "install/bin/pmvs2")

# define odm modules path
odm_modules_path = os.path.join(root_path, "build/bin")

# Define supported image extensions
supported_extensions = {'.jpg','.jpeg','.png'}

# Define the number of cores 
num_cores = multiprocessing.cpu_count()