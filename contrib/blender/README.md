# Blender scripts
# odm_photo
Renders photos from ODM generated texture models.
Currently can produce 360 panoramic photos and 360 3D panoramic (VR) photos.

## Requirements
* Blender
* ExifTool (must be on your PATH)

## Usage
To generate a 360 panoramic photo:

    blender -b photo_360.blend --python odm_photo.py -- <project-path>

Output is `<project-path>/odm_photo/odm_photo_360.jpg`.

To generate a 360 3D panoramic photo:

    blender -b photo_vr.blend --python odm_photo.py -- <project-path>

Output is `<project-path>/odm_photo/odm_photo_vr_L.jpg` and `<project-path>/odm_photo/odm_photo_vr_R.jpg`.

**NB: argument order matters!**
