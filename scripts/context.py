import os

# Define some needed locations
scripts_path = os.path.abspath(os.path.dirname(__file__))
root_path, _ = os.path.split(scripts_path)

superbuild_path = os.path.join(root_path, 'SuperBuild')
ccd_widths_path = os.path.join(root_path, 'ccd_defs.json')

pyopencv_path = os.path.join(superbuild_path, 'lib/python2.7/dist-packages')
opensfm_path = os.path.join(superbuild_path, "src/opensfm")


# Define supported image extensions
supported_extensions = {'.jpg','.jpeg'}