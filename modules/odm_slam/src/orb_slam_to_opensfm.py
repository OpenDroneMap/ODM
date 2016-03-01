import argparse
import json
import os
import yaml

import cv2
import numpy as np
from opensfm import transformations as tf
from opensfm.io import mkdir_p


SCALE = 50


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


def camera_from_config(video_filename, config_filename):
    '''
    Creates an OpenSfM from an ORB_SLAM2 config
    '''
    config = parse_orb_slam2_config_file(config_filename)
    fx = float(config['Camera.fx'])
    fy = float(config['Camera.fy'])
    cx = float(config['Camera.cx'])
    cy = float(config['Camera.cy'])
    k1 = float(config['Camera.k1'])
    k2 = float(config['Camera.k2'])
    p1 = float(config['Camera.p1'])
    p2 = float(config['Camera.p2'])
    width, height = get_video_size(video_filename)
    size = max(width, height)
    return {
        'width': width,
        'height': height,
        'focal': np.sqrt(fx * fy) / size,
        'k1': k1,
        'k2': k2
    }


def shot_id_from_timestamp(timestamp):
    T = 0.1  # TODO(pau) get this from config
    i = int(round(timestamp / T))
    return 'frame{0:06d}.png'.format(i)


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
        R = tf.quaternion_matrix([q[3], q[0], q[1], q[2]])[:3, :3].T
        t = -R.dot(c) * SCALE
        shot = {
            'camera': 'slamcam',
            'rotation': list(cv2.Rodrigues(R)[0].flat),
            'translation': list(t.flat),
            'created_at': timestamp,
        }
        shots[shot_id_from_timestamp(timestamp)] = shot
    return shots


def points_from_map_points(filename):
    points = {}
    with open(filename) as fin:
        lines = fin.readlines()

    for line in lines:
        words = line.split()
        point_id = words[1]
        coords = map(float, words[2:5])
        coords = [SCALE * i for i in coords]
        points[point_id] = {
            'coordinates': coords,
            'color': [100, 0, 200]
        }

    return points


def tracks_from_map_points(filename):
    tracks = []
    with open(filename) as fin:
        lines = fin.readlines()

    for line in lines:
        words = line.split()
        timestamp = float(words[0])
        shot_id = shot_id_from_timestamp(timestamp)
        point_id = words[1]
        row = [shot_id, point_id, point_id, '0', '0', '0', '0', '0']
        tracks.append('\t'.join(row))

    return '\n'.join(tracks)


def get_video_size(video):
    cap = cv2.VideoCapture(video)
    width = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return width, height


def extract_keyframes_from_video(video, reconstruction):
    '''
    Reads video and extracts a frame for each shot in reconstruction
    '''
    image_path = 'images'
    mkdir_p(image_path)
    T = 0.1  # TODO(pau) get this from config
    cap = cv2.VideoCapture(video)
    video_idx = 0

    shot_ids = sorted(reconstruction['shots'].keys())
    for shot_id in shot_ids:
        shot = reconstruction['shots'][shot_id]
        timestamp = shot['created_at']
        keyframe_idx = int(round(timestamp / T))

        while video_idx <= keyframe_idx:
            for i in range(20):
                ret, frame = cap.read()
                if ret:
                    break
                else:
                    print 'retrying'
            if not ret:
                raise RuntimeError(
                    'Cound not find keyframe {} in video'.format(shot_id))
            if video_idx == keyframe_idx:
                cv2.imwrite(os.path.join(image_path, shot_id), frame)
            video_idx += 1

    cap.release()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Convert ORB_SLAM2 output to OpenSfM')
    parser.add_argument(
        'video',
        help='the tracked video file')
    parser.add_argument(
        'trajectory',
        help='the trajectory file')
    parser.add_argument(
        'points',
        help='the map points file')
    parser.add_argument(
        'config',
        help='config file with camera calibration')
    args = parser.parse_args()

    r = {
        'cameras': {},
        'shots': {},
        'points': {},
    }

    r['cameras']['slamcam'] = camera_from_config(args.video, args.config)
    r['shots'] = shots_from_trajectory(args.trajectory)
    r['points'] = points_from_map_points(args.points)
    tracks = tracks_from_map_points(args.points)

    with open('reconstruction.json', 'w') as fout:
        json.dump([r], fout, indent=4)
    with open('tracks.csv', 'w') as fout:
        fout.write(tracks)

    extract_keyframes_from_video(args.video, r)
