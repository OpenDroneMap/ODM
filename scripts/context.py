import os

# Define third party libs location
scripts_path = os.path.abspath(os.path.dirname(__file__))
superbuild_path = os.path.join(scripts_path[:-7], 'SuperBuild/')

pyopencv_path = os.path.join(superbuild_path, 'lib/python2.7/dist-packages')
opensfm_path = os.path.join(superbuild_path, "src/opensfm")

# Define supported image extensions
supported_extensions = {'.jpg','.jpeg'}