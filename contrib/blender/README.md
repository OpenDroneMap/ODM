# Blender scripts
# odm_photo
Renders photos from ODM generated texture models.
Currently can produce 360 panoramic photos and 360 3D panoramic (VR) photos.
NB: the default resolution for 360 photos is 6000x3000 (maximum supported by Facebook).

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

# odm_video
Renders videos from ODM generated texture models.
Currently can produce 360 panoramic videos.
NB: the default resolution is 4096x2048 (maximum supported by Facebook).

## Requirements
* Blender
* Python 2.7 (must be on your PATH)
* Spatial Media Metadata Injector (https://github.com/google/spatial-media/tree/master/spatialmedia) (place in `spatialmedia` subdirectory)

## Usage
To generate a 360 panoramic photo:

    blender -b photo_360.blend --python odm_video.py -- <project-path> <camera-waypoints.xyz> <number-of-frames>

Output is `<project-path>/odm_video/odm_video_360.mp4`.
