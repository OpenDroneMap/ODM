#!/usr/bin/env python

# Renders a video.
# To generate a 360 panoramic video:
# blender -b photo_360.blend --python odm_video.py -- <project-path> <camera-waypoints.xyz> <number-of-frames>

import sys
import bpy
import subprocess
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
    bpy.ops.constraint.followpath_path_animate(animateContext, constraint='CameraFollowPath')

    blendName = bpy.path.display_name_from_filepath(bpy.data.filepath)
    fileName = projectHome + '/odm_video/odm_' + blendName.replace('photo', 'video')
    scene.frame_start = 0
    scene.frame_end = numFrames - 1
    render = scene.render
    render.filepath = fileName
    render.resolution_x = 4096
    render.resolution_y = 2048
    render.image_settings.file_format = 'FFMPEG'
    #bpy.ops.render.render(animation=True)


def loadWaypoints(filename):
    waypoints = []
    with open(filename) as f:
        for line in f:
           xyz = line.split()
           waypoints.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))
    return waypoints


if __name__ == '__main__':
    main()
