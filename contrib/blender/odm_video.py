#!/usr/bin/env python

# Renders a video.
# To generate a 360 panoramic video:
# blender -b photo_360.blend --python odm_video.py -- <project-path> <camera-waypoints.xyz> <number-of-frames>

import sys
import subprocess
import os
import bpy
from common import loadMesh


def main():

    if len(sys.argv) < 7 or sys.argv[-4] != '--':
        sys.exit('Please provide the ODM project path, camera waypoints (xyz format), and number of frames.')

    projectHome = sys.argv[-3]
    waypointFile = sys.argv[-2]
    numFrames = int(sys.argv[-1])

    loadMesh(projectHome +
             '/odm_texturing/odm_textured_model_geo.obj')

    waypoints = loadWaypoints(waypointFile)
    numWaypoints = len(waypoints)

    scene = bpy.data.scenes['Scene']

    # create path thru waypoints
    curve = bpy.data.curves.new(name='CameraPath', type='CURVE')
    curve.dimensions = '3D'
    curve.twist_mode = 'Z_UP'
    nurbs = curve.splines.new('NURBS')
    nurbs.points.add(numWaypoints-1)
    weight = 1
    for i in range(numWaypoints):
       nurbs.points[i].co[0] = waypoints[i][0]
       nurbs.points[i].co[1] = waypoints[i][1]
       nurbs.points[i].co[2] = waypoints[i][2]
       nurbs.points[i].co[3] = weight
    nurbs.use_endpoint_u = True
    path = bpy.data.objects.new(name='CameraPath', object_data=curve)
    scene.objects.link(path)

    camera = bpy.data.objects['Camera']
    camera.location[0] = 0
    camera.location[1] = 0
    camera.location[2] = 0
    followPath = camera.constraints.new(type='FOLLOW_PATH')
    followPath.name = 'CameraFollowPath'
    followPath.target = path
    followPath.use_curve_follow = True
    animateContext = bpy.context.copy()
    animateContext['constraint'] = followPath
    bpy.ops.constraint.followpath_path_animate(animateContext,
                                               constraint='CameraFollowPath',
                                               frame_start=0,
                                               length=numFrames)

    blendName = bpy.path.display_name_from_filepath(bpy.data.filepath)
    fileName = projectHome + '/odm_video/odm_' + blendName.replace('photo', 'video')
    scene.frame_start = 0
    scene.frame_end = numFrames
    render = scene.render
    render.filepath = fileName + '.mp4'
    render.image_settings.file_format = 'FFMPEG'
    if(render.use_multiview):
        render.image_settings.stereo_3d_format.display_mode = 'TOPBOTTOM'
        render.image_settings.views_format = 'STEREO_3D'
        render.views[0].file_suffix = ''
        format3d = 'top-bottom'
    else:
        width = render.resolution_x
        height = render.resolution_y
        format3d = 'none'
    render.resolution_x = 4096
    render.resolution_y = 2048

    render.ffmpeg.audio_codec = 'AAC'
    render.ffmpeg.codec = 'H264'
    render.ffmpeg.format = 'MPEG4'
    render.ffmpeg.video_bitrate = 45000
    bpy.ops.render.render(animation=True)

    writeMetadata(fileName+'.mp4', format3d)


def loadWaypoints(filename):
    waypoints = []
    with open(filename) as f:
        for line in f:
           xyz = line.split()
           waypoints.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))
    return waypoints


def writeMetadata(filename, format3d):
    subprocess.run(['python',
                    'spatialmedia',
                    '-i',
                    '--stereo='+format3d,
                    filename,
                    filename+'.injected'])
    # check metadata injector was successful
    if os.path.exists(filename+'.injected'):
        os.remove(filename)
        os.rename(filename+'.injected', filename)


if __name__ == '__main__':
    main()
