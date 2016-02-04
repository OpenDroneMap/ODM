import argparse
import json
import yaml

import cv2
import numpy as np
from opensfm import transformations as tf


parser = argparse.ArgumentParser(
    description='Convert ORB_SLAM2 output to OpenSfM')
parser.add_argument(
    'trajectory',
    help='the trajectory file')
parser.add_argument(
    'config',
    help='config file with camera calibration')
args = parser.parse_args()


def parse_orb_slam2_config_file(filename):
    '''
    Parse ORB_SLAM2 config file.

    Parsing manually since neither pyyaml nor cv2.FileStorage seem to work.
    '''
    res = {}
    with open(filename) as fin:
        lines = fin.readlines()

    for line in lines:
        line = line.strip()
        if line and line[0] != '#' and ':' in line:
            key, value = line.split(':')
            res[key.strip()] = value.strip()
    return res

config = parse_orb_slam2_config_file(args.config)


def camera_from_config(config):
    '''
    Creates an OpenSfM from an ORB_SLAM2 config
    '''
    fx = float(config['Camera.fx'])
    fy = float(config['Camera.fy'])
    cx = float(config['Camera.cx'])
    cy = float(config['Camera.cy'])
    k1 = float(config['Camera.k1'])
    k2 = float(config['Camera.k2'])
    p1 = float(config['Camera.p1'])
    p2 = float(config['Camera.p2'])
    width, height = 640, 480 # TODO(pau) get this from video file or keyframes
    size = max(width, height)
    return {
        'width': width,
        'height': height,
        'focal': np.sqrt(fx * fy) / size,
        'k1': k1,
        'k2': k2
    }

def shots_from_trajectory(trajectory_filename):
    '''
    Create opensfm shots from an orb_slam2/TUM trajectory
    '''
    shots = {}
    with open(trajectory_filename) as fin:
        lines = fin.readlines()

    for line in lines:
        a = map(float, line.split())
        timestamp = a[0]
        c = np.array(a[1:4])
        q = np.array(a[4:8])
        R = tf.quaternion_matrix(q)[:3, :3]
        t = -R.dot(c)
        shot = {
            'camera': 'slamcam',
            'rotation': list(cv2.Rodrigues(R)[0].flat),
            'translation': list(t.flat)
        }
        shots['frame{:12d}.png'.format(int(timestamp))] = shot
    return shots


r = {
    'cameras': {},
    'shots': {}
}

r['cameras']['slamcam'] = camera_from_config(config)
r['shots'] = shots_from_trajectory(args.trajectory)

with open('reconstruction.json', 'w') as fout:
    json.dump([r], fout, indent=4)
