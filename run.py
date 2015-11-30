#!/usr/bin/python

import os
import sys
import multiprocessing
import json
import datetime
import re
import subprocess
import shutil
import shlex
# import collections  # Never used
import argparse
import errno

import knnMatch_exif

# the defs
CURRENT_DIR = os.getcwd()
BIN_PATH_ABS = os.path.abspath(os.path.dirname(os.path.abspath(__file__)))
PYOPENCV_PATH = os.path.join(BIN_PATH_ABS, 'lib/python2.7/dist-packages')
OPENSFM_PATH = os.path.join(BIN_PATH_ABS, "src/OpenSfM")
CORES = multiprocessing.cpu_count()

def get_ccd_widths():
    """Return the CCD Width of the camera listed in the JSON defs file."""
    with open(BIN_PATH_ABS + '/ccd_defs.json') as jsonFile:
        return json.load(jsonFile)

objects = []
ccdWidths = get_ccd_widths()

BIN_PATH = BIN_PATH_ABS + '/bin'

objectStats = {
    'count': 0, 'good': 0, 'bad': 0, 'minWidth': 0, 'minHeight': 0,
    'maxWidth': 0, 'maxHeight': 0
    }

jobOptions = {'resizeTo': 0, 'srcDir': CURRENT_DIR, 'utmZone': -999,
              'utmSouth': False, 'utmEastOffset': 0, 'utmNorthOffset': 0}

# parse arguments
processopts = ['resize', 'opensfm', 'getKeypoints', 'match', 'bundler', 'cmvs', 'pmvs',
               'odm_meshing', 'odm_texturing', 'odm_georeferencing',
               'odm_orthophoto']

parser = argparse.ArgumentParser(description='OpenDroneMap')
parser.add_argument('--resize-to',  # currently doesn't support 'orig'
                    metavar='<integer>',
                    default=2400,
                    type=int,
                    help='Resizes images by the largest side. Default: %(default)s')

parser.add_argument('--job-dir-name',
                    metavar='<string>',
                    type=str,
                    help='Name of the output folder, only ASCII charaters can be used. If not provided "reconstruction-with-image-size-[resize-to]" will be used')

parser.add_argument('--start-with', '-s',
                    metavar='<task>',
                    default='resize',
                    choices=processopts,
                    help=('Start with the provided task. Can be one of: ' + ' | '.join(processopts)))

parser.add_argument('--end-with', '-e',
                    metavar='<task>',
                    default='odm_orthophoto',
                    choices=processopts,
                    help=('Finish at the provided task. Can be one of: ' + ' | '.join(processopts)))

parser.add_argument('--run-only',
                    metavar='<task>',
                    choices=processopts,
                    help=('Run only one task. Can be one of: ' + ' | '.join(processopts)))

use_opensfm_parser = parser.add_mutually_exclusive_group(required=False)
use_opensfm_parser.add_argument('--use-opensfm',
                    dest='use_opensfm',
                    action='store_true',
                    help='use OpenSfM to find the camera positions '
                         '(replaces getKeypoints, match and bundler steps)')
use_opensfm_parser.add_argument('--use-bundler',
                    dest='use_opensfm',
                    action='store_false',
                    help='use Bundler to find the camera positions.')
parser.set_defaults(use_opensfm=True)

parser.add_argument('--force-focal',
                    metavar='<positive float>',
                    type=float,
                    help=('Override the focal length information for the '
                            'images'))

parser.add_argument('--force-ccd',
                    metavar='<positive float>',
                    type=float,
                    help='Override the ccd width information for the images')

parser.add_argument('--min-num-features',
                    metavar='<integer>',
                    default=4000,
                    type=int,
                    help=('Minimum number of features to extract per image. '
                          'More features leads to better results but slower '
                          'execution. Default: %(default)s'))

parser.add_argument('--matcher-threshold',
                    metavar='<percent>',
                    default=2.0,
                    type=float,
                    help=('Ignore matched keypoints if the two images share '
                            'less than <float> percent of keypoints. Default: %(default)s'))

parser.add_argument('--matcher-ratio',
                    metavar='<float>',
                    default=0.6,
                    type=float,
                    help=('Ratio of the distance to the next best matched '
                            'keypoint. Default: %(default)s'))

parser.add_argument('--matcher-neighbors',
                    metavar='<integer>',
                    default=8,
                    type=int,
                    help='Number of nearest images to pre-match based on GPS exif data. '
                            'Set to 0 to skip pre-matching. '
                            'Neighbors works together with Distance parameter, '
                            'set both to 0 to not use pre-matching. OpenSFM uses both parameters at the same time, '
                            'Bundler uses only one which has value, prefering the Neighbors parameter.'
                            ' Default: %(default)s')

parser.add_argument('--matcher-distance',
                    metavar='<integer>',
                    default=0,
                    type=int,
                    help='Distance threshold in meters to find pre-matching images based on GPS exif data. '
                            'Set to 0 to skip pre-matching.'
                            ' Default: %(default)s')

parser.add_argument('--cmvs-maxImages',
                    metavar='<integer>',
                    default=500,
                    type=int,
                    help='The maximum number of images per cluster. Default: %(default)s')

parser.add_argument('--pmvs-level',
                    metavar='<positive integer>',
                    default=1,
                    type=int,
                    help=('The level in the image pyramid that is used '
                            'for the computation. see '
                            'http://www.di.ens.fr/pmvs/documentation.html for '
                            'more pmvs documentation. Default: %(default)s'))

parser.add_argument('--pmvs-csize',
                    metavar='< positive integer>',
                    default=2,
                    type=int,
                    help='Cell size controls the density of reconstructions. Default: %(default)s')

parser.add_argument('--pmvs-threshold',
                    metavar='<float: -1.0 <= x <= 1.0>',
                    default=0.7,
                    type=float,
                    help=('A patch reconstruction is accepted as a success '
                            'and kept, if its associcated photometric consistency '
                            'measure is above this threshold. Default: %(default)s'))

parser.add_argument('--pmvs-wsize',
                    metavar='<positive integer>',
                    default=7,
                    type=int,
                    help=('pmvs samples wsize x wsize pixel colors from '
                            'each image to compute photometric consistency '
                            'score. For example, when wsize=7, 7x7=49 pixel '
                            'colors are sampled in each image. Increasing the '
                            'value leads to more stable reconstructions, but '
                            'the program becomes slower. Default: %(default)s'))

parser.add_argument('--pmvs-minImageNum',
                    metavar='<positive integer>',
                    default=3,
                    type=int,
                    help=('Each 3D point must be visible in at least '
                            'minImageNum images for being reconstructed. 3 is '
                            'suggested in general.'))

parser.add_argument('--odm_meshing-maxVertexCount',
                    metavar='<positive integer>',
                    default=100000,
                    type=int,
                    help='The maximum vertex count of the output mesh. Default: %(default)s')

parser.add_argument('--odm_meshing-octreeDepth',
                    metavar='<positive integer>',
                    default=9,
                    type=int,
                    help=('Oct-tree depth used in the mesh reconstruction, '
                            'increase to get more vertices, recommended '
                            'values are 8-12, the default is %(default)s'))

parser.add_argument('--odm_meshing-samplesPerNode',
                    metavar='<float >= 1.0>',
                    default=1.0,
                    type=float,
                    help=('Number of points per octree node, recommended '
                            ' and default value: %(default)s'))

parser.add_argument('--odm_meshing-solverDivide',
                    metavar='<positive integer>',
                    default=9,
                    type=int,
                    help=('Oct-tree depth at which the Laplacian equation '
                            'is solved in the surface reconstruction step. '
                            'Increasing this value increases computation '
                            'times slightly but helps reduce memory usage. Default: %(default)s'))

parser.add_argument('--odm_texturing-textureResolution',
                    metavar='<positive integer>',
                    default=4096,
                    type=int,
                    help=('The resolution of the output textures. Must be '
                            'greater than textureWithSize. Default: %(default)s'))

parser.add_argument('--odm_texturing-textureWithSize',
                    metavar='<positive integer>',
                    default=3600,
                    type=int,
                    help=('The resolution to rescale the images performing '
                            'the texturing. Default: %(default)s'))

parser.add_argument('--odm_georeferencing-gcpFile',
                    metavar='<path string>',
                    default='gcp_list.txt',
                    help=('path to the file containing the ground control '
                            'points used for georeferencing. The default file is %(default)s. The file needs to '
                            'be on the following line format: \neasting '
                            'northing height pixelrow pixelcol imagename'))

parser.add_argument('--odm_georeferencing-useGcp',
                    action='store_true',
                    default=False,
                    help = 'Enabling GCPs from the file above. The GCP file is not used by default.')

parser.add_argument('--odm_orthophoto-resolution',
                    metavar='<float > 0.0>',
                    default=20.0,
                    type=float,
                    help=('Orthophoto ground resolution in pixels/meter. Default: %(default)s'))

parser.add_argument('--zip-results',
                    action='store_true',
                    default=False,
                    help='Compress the results using gunzip')

args = parser.parse_args()

print "\n  - configuration:"

# print vars(args)
for arg in vars(args):
    print "    ", arg, ":", getattr(args, arg)


def run(cmd):
    """Run a system command"""
    print 'running', cmd
    returnCode = os.system(cmd)
    print 'b'
    if (returnCode != 0):
        sys.exit("\nquitting cause: \n\t" + cmd + "\nreturned with code " +
                 str(returnCode) + ".\n")


def now():
    """Return the current time"""
    return datetime.datetime.now().strftime('%a %b %d %H:%M:%S %Z %Y')


def run_and_return(cmdSrc, cmdDest):
    """Run a system command and return the output"""
    srcProcess = subprocess.Popen(shlex.split(cmdSrc), stdout=subprocess.PIPE)
    if cmdDest:
        destProcess = subprocess.Popen(shlex.split(cmdDest),
                                       stdin=srcProcess.stdout,
                                       stdout=subprocess.PIPE)
        stdout, stderr = destProcess.communicate()
    else:
        stdout, stderr = srcProcess.communicate()

    return stdout.decode('ascii')


def mkdir_p(path):
    '''Make a directory including parent directories.
    '''
    try:
        os.makedirs(path)
    except os.error as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise


def calculate_EPSG(utmZone, south):
    """Calculate and return the EPSG"""
    if south:
        return 32700 + utmZone
    else:
        return 32600 + utmZone


def parse_coordinate_system():
    """Write attributes to jobOptions from coord file"""
    if os.path.isfile(jobOptions['jobDir'] +
                      '/odm_georeferencing/coordFile.txt'):
        with open(jobOptions['jobDir'] + '/odm_georeferencing/coordFile.txt') as f:
            for lineNumber, line in enumerate(f):
                if lineNumber == 0:
                    # check for the WGS84 UTM 17N format
                    match_wgs_utm = re.search('WGS84 UTM (\d{1,2})(N|S)',line.strip(),re.I)
                    if match_wgs_utm:
                        utmZoneString = match_wgs_utm.group(1)
                        utmSouthBool = ( match_wgs_utm.group(2).upper() == 'S' )
                        jobOptions['csString'] = '+datum=WGS84 +proj=utm +zone=' + utmZoneString + (' +south' if utmSouthBool else '')
                        jobOptions['epsg'] = calculate_EPSG(int(utmZoneString), utmSouthBool)
                    else:
                        jobOptions['csString'] = line.strip()
                elif lineNumber == 1:
                    tokens = line.split(' ')
                    if len(tokens) == 2:
                        jobOptions['utmEastOffset'] = int(tokens[0].strip())
                        jobOptions['utmNorthOffset'] = int(tokens[1].strip())
                else:
                    break

def coord_to_fractions(coord,refs):
    deg_dec = abs(float(coord))
    deg = int(deg_dec)
    minute_dec = (deg_dec-deg)*60
    minute = int(minute_dec)

    sec_dec = (minute_dec-minute)*60
    sec_dec = round(sec_dec,3)
    sec_denominator = 1000
    sec_numerator = int(sec_dec*sec_denominator)
    if float(coord) >= 0 :
        latRef = refs[0]
    else:
        latRef = refs[1]

    output = str(deg) + '/1 ' + str(minute) + '/1 ' + str(sec_numerator) + '/' + str(sec_denominator)
    return (output, latRef)

def prepare_objects():
    """Prepare the jobOptions and fileObjects dicts"""
    source_files = run_and_return('ls -1', 'egrep "\.[jJ]{1}[pP]{1}[eE]{0,1}[gG]{1}"')

    print "\n  - source files - " + now()

    for filename in source_files.split('\n'):
        filename = filename.rstrip('\n')
        if not filename:
            continue
        file_make = run_and_return('jhead "' + filename + '"', 'grep "Camera make"')
        file_model = run_and_return('jhead "' + filename + '"', 'grep "Camera model"')
        file_focal = run_and_return('jhead "' + filename + '"', 'grep "Focal length"')
        file_ccd = run_and_return('jhead "' + filename + '"', 'grep "CCD width"')
        file_resolution = run_and_return('jhead "' + filename + '"', 'grep "Resolution"')

        fileObject = {}

        fileObject["src"] = filename
        fileObject["base"] = re.sub("\.[^\.]*$", "", filename)

        match = re.search(": ([^\n\r]*)", file_make)
        if match:
            fileObject["make"] = match.group(1).strip()

        match = re.search(": ([^\n\r]*)", file_model)
        if match:
            fileObject["model"] = match.group(1).strip()

        if "make" in fileObject:
            fileObject["make"] = re.sub("^\s+", "", fileObject["make"])
            fileObject["make"] = re.sub("\s+$", "", fileObject["make"])

        if "model" in fileObject:
            fileObject["model"] = re.sub("^\s+", "", fileObject["model"])
            fileObject["model"] = re.sub("\s+$", "", fileObject["model"])

        if "make" in fileObject:
            fileObject["id"] = fileObject["make"]
        if "model" in fileObject:
            fileObject["id"] += " " + fileObject["model"]

        match = re.search(": ([0-9]*) x ([0-9]*)", file_resolution)
        if match:
            fileObject["width"] = int(match.group(1).strip())
            fileObject["height"] = int(match.group(2).strip())

        if args.force_focal is None:
            match = re.search(":[\ ]*([0-9\.]*)mm", file_focal)
            if match:
                fileObject["focal"] = float((match.group()[1:-2]).strip())
        else:
            fileObject["focal"] = args.force_focal

        if args.force_ccd is None:
            match = re.search(":[\ ]*([0-9\.]*)mm", file_ccd)
            if match:
                fileObject["ccd"] = float(match.group()[1:-2].strip())

            if ("ccd" not in fileObject) and ("id" in fileObject):
                fileObject["ccd"] = float(ccdWidths[fileObject["id"]])
        else:
            fileObject["ccd"] = args.force_ccd

        if "ccd" in fileObject and "focal" in fileObject and "width" in fileObject and "height" in fileObject:
            if fileObject["width"] > fileObject["height"]:
                fileObject["focalpx"] = fileObject["width"] * fileObject["focal"] / fileObject["ccd"]
            else:
                fileObject["focalpx"] = fileObject["height"] * fileObject["focal"] / fileObject["ccd"]

            fileObject["isOk"] = True
            objectStats["good"] += 1

            print "     using " + fileObject["src"] + "     dimensions: " + str(fileObject["width"]) + "x" + str(fileObject["height"]) + " / focal: " + str(fileObject["focal"]) + "mm / ccd: " + str(fileObject["ccd"]) + "mm"
        else:
            fileObject["isOk"] = False
            objectStats["bad"] += 1

            if "id" in fileObject:
                print "\n    no CCD width or focal length found for " + fileObject["src"] + " - camera: \"" + fileObject["id"] + "\""
            else:
                print "\n    no CCD width or focal length found"

        objectStats["count"] += 1

        if "width" in fileObject and "height" in fileObject:
            if objectStats["minWidth"] == 0:
                objectStats["minWidth"] = fileObject["width"]
            if objectStats["minHeight"] == 0:
                objectStats["minHeight"] = fileObject["height"]

            if objectStats["minWidth"] < fileObject["width"]:
                objectStats["minWidth"] = objectStats["minWidth"]
            else:
                objectStats["minWidth"] = fileObject["width"]

            if objectStats["minHeight"] < fileObject["height"]:
                objectStats["minHeight"] = objectStats["minHeight"]
            else:
                objectStats["minHeight"] = fileObject["height"]

            if objectStats["maxWidth"] > fileObject["width"]:
                objectStats["maxWidth"] = objectStats["maxWidth"]
            else:
                objectStats["maxWidth"] = fileObject["width"]

            if objectStats["maxHeight"] > fileObject["height"]:
                objectStats["maxHeight"] = objectStats["maxHeight"]
            else:
                objectStats["maxHeight"] = fileObject["height"]

        objects.append(fileObject)

    if "good" not in objectStats:
        print "\n    found no usable images - quitting\n"
        sys.exit()
    else:
        print "\n    found " + str(objectStats["good"]) + " usable images"

    print "\n"

    jobOptions["resizeTo"] = args.resize_to

    print "  using max image size of " + str(jobOptions["resizeTo"]) + " x " + str(jobOptions["resizeTo"])

    if args.job_dir_name is None:
        args.job_dir_name = "reconstruction-with-image-size-" + str(jobOptions["resizeTo"])

    jobOptions["jobDir"] = jobOptions["srcDir"] + "/" + str(args.job_dir_name)

    jobOptions["step_1_convert"] = jobOptions["jobDir"] + "/_convert.templist.txt"
    jobOptions["step_1_vlsift"] = jobOptions["jobDir"] + "/_vlsift.templist.txt"
    jobOptions["step_1_gzip"] = jobOptions["jobDir"] + "/_gzip.templist.txt"

    jobOptions["step_2_filelist"] = jobOptions["jobDir"] + "/_filelist.templist.txt"
    jobOptions["step_2_macthes_jobs"] = jobOptions["jobDir"] + "/_matches_jobs.templist.txt"
    jobOptions["step_2_matches_dir"] = jobOptions["jobDir"] + "/matches"
    jobOptions["step_2_matches"] = jobOptions["jobDir"] + "/matches.init.txt"

    jobOptions["step_3_filelist"] = jobOptions["jobDir"] + "/list.txt"
    jobOptions["step_3_bundlerOptions"] = jobOptions["jobDir"] + "/options.txt"

    try:
        os.mkdir(jobOptions["jobDir"])
    except:
        pass

    for fileObject in objects:
        if fileObject["isOk"]:
            fileObject["step_0_resizedImage"] = jobOptions["jobDir"] + "/" + fileObject["src"]
            fileObject["step_1_pgmFile"] = jobOptions["jobDir"] + "/" + fileObject["base"] + ".pgm"
            fileObject["step_1_keyFile"] = jobOptions["jobDir"] + "/" + fileObject["base"] + ".key"
            fileObject["step_1_gzFile"] = jobOptions["jobDir"] + "/" + fileObject["base"] + ".key.gz"


def resize():
    """Resize images"""
    print "\n  - preparing images - " + now()

    os.chdir(jobOptions["jobDir"])

    for fileObject in objects:
        if fileObject["isOk"]:
            if not os.path.isfile(fileObject["step_0_resizedImage"]):
                if jobOptions["resizeTo"] != 0 and ((int(fileObject["width"]) > jobOptions["resizeTo"]) or (fileObject["height"] > jobOptions["resizeTo"])):
                    sys.stdout.write("    resizing " + fileObject["src"] + " \tto " + fileObject["step_0_resizedImage"])
                    run("convert -resize " + str(jobOptions["resizeTo"]) + "x" + str(jobOptions["resizeTo"]) + " -quality 100 \"" + jobOptions["srcDir"] + "/" + fileObject["src"] + "\" \"" + fileObject["step_0_resizedImage"] + "\"")
                else:
                    sys.stdout.write("     copying " + fileObject["src"] + " \tto " + fileObject["step_0_resizedImage"])
                    shutil.copyfile(CURRENT_DIR + "/" + fileObject["src"], fileObject["step_0_resizedImage"])
            else:
                print "     using existing " + fileObject["src"] + " \tto " + fileObject["step_0_resizedImage"]

            file_resolution = run_and_return('jhead "' + fileObject["step_0_resizedImage"] + '"', 'grep "Resolution"')
            match = re.search(": ([0-9]*) x ([0-9]*)", file_resolution)
            if match:
                fileObject["width"] = int(match.group(1).strip())
                fileObject["height"] = int(match.group(2).strip())
            print "\t (" + str(fileObject["width"]) + " x " + str(fileObject["height"]) + ")"


def getKeypoints():
    """Run vlsift to create keypoint files for each image"""
    print "\n  - finding keypoints - " + now()

    os.chdir(jobOptions["jobDir"])

    vlsiftJobs = ""
    c = 0

    for fileObject in objects:
        c += 1

        if fileObject["isOk"]:
            if not os.path.isfile(jobOptions["jobDir"] + "/" + fileObject["base"] + ".key.bin"):
                vlsiftJobs += "echo -n \"     " + str(c).zfill(len(str(objectStats["good"]))) + "/" + str(objectStats["good"]) + " - \" && convert -format pgm \"" + fileObject["step_0_resizedImage"] + "\" \"" + fileObject["step_1_pgmFile"] + "\""
                vlsiftJobs += " && \"" + BIN_PATH + "/vlsift\" \"" + fileObject["step_1_pgmFile"] + "\" -o \"" + fileObject["step_1_keyFile"] + ".sift\" > /dev/null && perl \"" + BIN_PATH + "/../convert_vlsift_to_lowesift.pl\" \"" + jobOptions["jobDir"] + "/" + fileObject["base"] + "\""
                vlsiftJobs += " && gzip -f \"" + fileObject["step_1_keyFile"] + "\""
                vlsiftJobs += " && rm -f \"" + fileObject["step_1_pgmFile"] + "\""
                vlsiftJobs += " && rm -f \"" + fileObject["step_1_keyFile"] + ".sift\""
                vlsiftJobs += " && echo \"\"\n"
            else:
                print "     using existing " + jobOptions["jobDir"] + "/" + fileObject["base"] + ".key.bin"

    siftDest = open(jobOptions["step_1_vlsift"], 'w')
    siftDest.write(vlsiftJobs)
    siftDest.close()

    run("\"" + BIN_PATH + "/parallel\" --no-notice --halt-on-error 1 -j+0 < \"" + jobOptions["step_1_vlsift"] + "\"")


def match():
    """Run matches on images"""
    print "\n  - matching keypoints - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["step_2_matches_dir"])
    except:
        pass

    matchesJobs = ""
    c = 0
    t = (objectStats["good"] - 1) * (objectStats["good"] / 2)  # BUG:unused

    preselected_pairs = []

    # Create a file list with all keypoint files
    filesList = ""
    for fileObject in objects:
        if fileObject["isOk"]:
            filesList += fileObject["step_1_keyFile"] + "\n"
    matchDest = open(jobOptions["step_2_filelist"], 'w')
    matchDest.write(filesList)
    matchDest.close()

    # try to do preselection
    do_preselection = False
    if args.matcher_neighbors > 0 or args.matcher_distance > 0:
        do_preselection = True
        if args.matcher_neighbors > 0:
            k_distance = args.matcher_neighbors
            use_knn_mode = True
        else:
            k_distance = args.matcher_distance
            use_knn_mode = False
        preselected_pairs = knnMatch_exif.preselect_pairs(BIN_PATH + "/odm_extract_utm", jobOptions["step_2_filelist"], k_distance, use_knn_mode)

    if len(preselected_pairs) != 0:
        # preselection was succesfull
        for i, j, in preselected_pairs:
            c += 1
            if i < 10:
                print i, j
            if not os.path.isfile(jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt"):
                matchesJobs += "echo -n \".\" && touch \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" && \"" + BIN_PATH + "/KeyMatch\" \"" + objects[i]["step_1_keyFile"] + "\" \"" + objects[j]["step_1_keyFile"] + "\" \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" " + str(args.matcher_ratio) + " " + str(args.matcher_threshold) + "\n"
    else:
        # preselection failed, Match all image pairs
        if do_preselection:
            print "Failed to run pair preselection, proceeding with exhaustive matching."
        for i in range(0, objectStats["good"]):
            for j in range(i + 1, objectStats["good"]):
                c += 1
                if not os.path.isfile(jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt"):
                    matchesJobs += "echo -n \".\" && touch \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" && \"" + BIN_PATH + "/KeyMatch\" \"" + objects[i]["step_1_keyFile"] + "\" \"" + objects[j]["step_1_keyFile"] + "\" \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" " + str(args.matcher_ratio) + " " + str(args.matcher_threshold) + "\n"

    matchDest = open(jobOptions["step_2_macthes_jobs"], 'w')
    matchDest.write(matchesJobs)
    matchDest.close()

    run("\"" + BIN_PATH + "/parallel\" --no-notice --halt-on-error 1 -j+0 < \"" + jobOptions["step_2_macthes_jobs"] + "\"")
    run("rm -f \"" + jobOptions["step_2_matches"] + "\"")

    for i in range(0, objectStats["good"]):
        for j in range(i + 1, objectStats["good"]):
            c += 1
            if os.path.isfile(jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt") and os.path.getsize(jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt") > 0:
                run("echo \"" + str(i) + " " + str(j) + "\" >> \"" + jobOptions["step_2_matches"] + "\" && cat \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" >> \"" + jobOptions["step_2_matches"] + "\"")

    filesList = ""
    for fileObject in objects:
        if fileObject["isOk"]:
            filesList += fileObject["step_1_keyFile"] + "\n"

    matchDest = open(jobOptions["step_2_filelist"], 'w')
    matchDest.write(filesList)
    matchDest.close()

#   run("\"" + BIN_PATH + "/KeyMatchFull\" \"" + jobOptions["step_2_filelist"] + "\" \"" + jobOptions["step_2_matches"] + "\"    ")


def bundler():
    """Run bundler and prepare bundle for PMVS"""
    print "\n  - running bundler - " + now()

    os.chdir(jobOptions["jobDir"])

    try:
        os.mkdir(jobOptions["jobDir"] + "/bundle")
    except:
        pass
    try:
        os.mkdir(jobOptions["jobDir"] + "/pmvs")
    except:
        pass
    try:
        os.mkdir(jobOptions["jobDir"] + "/pmvs/txt")
    except:
        pass
    try:
        os.mkdir(jobOptions["jobDir"] + "/pmvs/visualize")
    except:
        pass
    try:
        os.mkdir(jobOptions["jobDir"] + "/pmvs/models")
    except:
        pass

    filesList = ""

    for fileObject in objects:
        if fileObject["isOk"]:
            filesList += "./" + fileObject["src"] + " 0 {:.5f}\n".format(fileObject["focalpx"])

    filesList = filesList.rstrip('\n')

    bundlerOptions = "--match_table matches.init.txt\n"
    bundlerOptions += "--output bundle.out\n"
    bundlerOptions += "--output_all bundle_\n"
    bundlerOptions += "--output_dir bundle\n"
    bundlerOptions += "--variable_focal_length\n"
    bundlerOptions += "--use_focal_estimate\n"
    bundlerOptions += "--constrain_focal\n"
    bundlerOptions += "--constrain_focal_weight 0.0\n"
    bundlerOptions += "--estimate_distortion\n"
    bundlerOptions += "--run_bundle"

    run("echo \"" + bundlerOptions + "\" > \"" + jobOptions["step_3_bundlerOptions"] + "\"")

    bundlerDest = open(jobOptions["step_3_filelist"], 'w')
    bundlerDest.write(filesList)
    bundlerDest.close()

    run("\"" + BIN_PATH + "/bundler\" \"" + jobOptions["step_3_filelist"] + "\" --options_file \"" + jobOptions["step_3_bundlerOptions"] + "\" > bundle/out")
    run("\"" + BIN_PATH + "/Bundle2PMVS\" \"" + jobOptions["step_3_filelist"] + "\" bundle/bundle.out")
    run("\"" + BIN_PATH + "/RadialUndistort\" \"" + jobOptions["step_3_filelist"] + "\" bundle/bundle.out pmvs")

    i = 0
    for fileObject in objects:
        if fileObject["isOk"]:
            if os.path.isfile("pmvs/" + fileObject["base"] + ".rd.jpg"):
                nr = "{0:08d}".format(i)
                i += 1

                run("mv pmvs/" + fileObject["base"] + ".rd.jpg pmvs/visualize/" + str(nr) + ".jpg")
                run("mv pmvs/" + str(nr) + ".txt pmvs/txt/" + str(nr) + ".txt")

    run("\"" + BIN_PATH + "/Bundle2Vis\" pmvs/bundle.rd.out pmvs/vis.dat")


def opensfm():
    print "\n  - running OpenSfM - " + now()

    os.chdir(jobOptions["jobDir"])

    # Create bundler's list.txt
    filesList = ""
    for fileObject in objects:
        if fileObject["isOk"]:
            filesList += "./" + fileObject["src"] + " 0 {:.5f}\n".format(fileObject["focalpx"])
    filesList = filesList.rstrip('\n')

    with open(jobOptions["step_3_filelist"], 'w') as fout:
        fout.write(filesList)

    # Create opensfm working folder
    mkdir_p("opensfm")

    # Configure OpenSfM
    config = [
       "use_exif_size: no",
       "feature_process_size: {}".format(jobOptions["resizeTo"]),
       "feature_min_frames: {}".format(args.min_num_features),
       "processes: {}".format(CORES),
       "matching_gps_neighbors: {}".format(args.matcher_neighbors),
    ]

    if args.matcher_distance>0:
        config.append("matching_gps_distance: {}".format(args.matcher_distance))

    with open('opensfm/config.yaml', 'w') as fout:
        fout.write("\n".join(config))

    print 'running import_bundler'
    # Convert bundler's input to opensfm
    run('PYTHONPATH={} "{}/bin/import_bundler" opensfm --list list.txt'.format(PYOPENCV_PATH, OPENSFM_PATH))

    # Run OpenSfM reconstruction
    run('PYTHONPATH={} "{}/bin/run_all" opensfm'.format(PYOPENCV_PATH, OPENSFM_PATH))

    # Convert back to bundler's format
    run('PYTHONPATH={} "{}/bin/export_bundler" opensfm'.format(PYOPENCV_PATH, OPENSFM_PATH))

    bundler_to_pmvs("opensfm/bundle_r000.out")



def bundler_to_pmvs(bundle_out):
    """Converts bundler's output to PMVS format"""
    print "\n  - converting bundler output to PMVS - " + now()

    os.chdir(jobOptions['jobDir'])

    mkdir_p(jobOptions['jobDir'] + "/pmvs")
    mkdir_p(jobOptions['jobDir'] + "/pmvs/txt")
    mkdir_p(jobOptions['jobDir'] + "/pmvs/visualize")
    mkdir_p(jobOptions['jobDir'] + "/pmvs/models")

    run("\"" + BIN_PATH + "/Bundle2PMVS\" \"" + jobOptions["step_3_filelist"] + "\" " + bundle_out)
    run("\"" + BIN_PATH + "/RadialUndistort\" \"" + jobOptions["step_3_filelist"] + "\" " + bundle_out + " pmvs")

    i = 0
    for fileObject in objects:
        if fileObject["isOk"]:
            if os.path.isfile("pmvs/" + fileObject["base"] + ".rd.jpg"):
                nr = "{0:08d}".format(i)
                i += 1

                run("mv pmvs/" + fileObject["base"] + ".rd.jpg pmvs/visualize/" + str(nr) + ".jpg")
                run("mv pmvs/" + str(nr) + ".txt pmvs/txt/" + str(nr) + ".txt")

    run("\"" + BIN_PATH + "/Bundle2Vis\" pmvs/bundle.rd.out pmvs/vis.dat")


def cmvs():
    """Run CMVS"""
    print "\n  - running cmvs - " + now()

    os.chdir(jobOptions["jobDir"])

    run("\"" + BIN_PATH + "/cmvs\" pmvs/ " + str(args.cmvs_maxImages) + " " + str(CORES))
    run("\"" + BIN_PATH + "/genOption\" pmvs/ " + str(args.pmvs_level) + " " + str(args.pmvs_csize) + " " + str(args.pmvs_threshold) + " " + str(args.pmvs_wsize) + " " + str(args.pmvs_minImageNum) + " " + str(CORES))


def pmvs():
    """Run PMVS"""
    print "\n  - running pmvs - " + now()

    os.chdir(jobOptions["jobDir"])

    run("\"" + BIN_PATH + "/pmvs2\" pmvs/ option-0000")

    run("cp -Rf \"" + jobOptions["jobDir"] + "/pmvs/models\" \"" + jobOptions["jobDir"] + "-results\"")


def odm_meshing():
    """Run odm_meshing"""
    print "\n  - running meshing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_meshing")
    except:
        pass

    run("\"" + BIN_PATH + "/odm_meshing\" -inputFile " + jobOptions["jobDir"] + "-results/option-0000.ply -outputFile " + jobOptions["jobDir"] + "-results/odm_mesh-0000.ply -logFile " + jobOptions["jobDir"] + "/odm_meshing/odm_meshing_log.txt -maxVertexCount " + str(args.odm_meshing_maxVertexCount) + " -octreeDepth " + str(args.odm_meshing_octreeDepth) + " -samplesPerNode " + str(args.odm_meshing_samplesPerNode) + " -solverDivide " + str(args.odm_meshing_solverDivide))


def odm_texturing():
    """Run odm_texturing"""
    print "\n  - running texturing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_texturing")
        os.mkdir(jobOptions["jobDir"] + "-results/odm_texturing")
    except:
        pass

    # Create list of original image files
    pmvs_list = jobOptions["jobDir"] + "/pmvs/list.rd.txt"
    texturing_list = jobOptions['jobDir'] + "/odm_texturing/image_list.txt"
    with open(pmvs_list) as fin:
        with open(texturing_list, "w") as fout:
            for line in fin:
                base = line.rstrip('\n')[2:-4]
                for fileObject in objects:
                    if fileObject["base"] == base:
                        fout.write("./{}\n".format(fileObject["src"]))
                        break

    run("\"" + BIN_PATH + "/odm_texturing\" -bundleFile " + jobOptions["jobDir"] + "/pmvs/bundle.rd.out -imagesPath " + jobOptions["srcDir"] + "/ -imagesListPath " + texturing_list + " -inputModelPath " + jobOptions["jobDir"] + "-results/odm_mesh-0000.ply -outputFolder " + jobOptions["jobDir"] + "-results/odm_texturing/ -textureResolution " + str(args.odm_texturing_textureResolution) + " -bundleResizedTo " + str(jobOptions["resizeTo"]) + " -textureWithSize " + str(args.odm_texturing_textureWithSize) + " -logFile " + jobOptions["jobDir"] + "/odm_texturing/odm_texturing_log.txt")


def odm_georeferencing():
    """Run odm_georeferencing"""
    print "\n  - running georeferencing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_georeferencing")
    except:
        pass

    if not args.odm_georeferencing_useGcp:
        run("\"" + BIN_PATH + "/odm_extract_utm\" -imagesPath " + jobOptions["srcDir"] + "/ -imageListFile " + jobOptions["jobDir"] + "/pmvs/list.rd.txt -outputCoordFile " + jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt")
        run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + jobOptions["jobDir"] + "/pmvs/bundle.rd.out -inputCoordFile " + jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt -inputFile " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model.obj -outputFile " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo.obj -inputPointCloudFile " + jobOptions["jobDir"] + "-results/option-0000.ply -outputPointCloudFile " + jobOptions["jobDir"] + "-results/option-0000_georef.ply -logFile " + jobOptions["jobDir"] + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo_georef_system.txt")
    elif os.path.isfile(jobOptions["srcDir"] + "/" + args.odm_georeferencing_gcpFile):
        run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + jobOptions["jobDir"] + "/pmvs/bundle.rd.out -gcpFile " + jobOptions["srcDir"] + "/" + args.odm_georeferencing_gcpFile + " -imagesPath " + jobOptions["srcDir"] + "/ -imagesListPath " + jobOptions["jobDir"] + "/pmvs/list.rd.txt -bundleResizedTo " + str(jobOptions["resizeTo"]) + " -inputFile " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model.obj -outputFile " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo.obj -outputCoordFile " + jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt -inputPointCloudFile " + jobOptions["jobDir"] + "-results/option-0000.ply -outputPointCloudFile " + jobOptions["jobDir"] + "-results/option-0000_georef.ply -logFile " + jobOptions["jobDir"] + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo_georef_system.txt")
    else:
        print "Warning: No GCP file. Consider rerunning with argument --odm_georeferencing-useGcp false --start-with odm_georeferencing"
        print "Skipping orthophoto"
        args.end_with = "odm_georeferencing"

    if "csString" not in jobOptions:
        parse_coordinate_system()

    if "csString" in jobOptions and "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions:
        images = []
        with open(jobOptions["jobDir"] + "/pmvs/list.rd.txt") as f:
            images = f.readlines()

        if len(images) > 0:
            with open(jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt") as f:
                for lineNumber, line in enumerate(f):
                    if lineNumber >= 2 and lineNumber - 2 < len(images):
                        tokens = line.split(' ')
                        if len(tokens) >= 3:
                            x = float(tokens[0])
                            y = float(tokens[1])
                            z = float(tokens[2])
                            filename = images[lineNumber - 2]
                            coords_wgs84_out = run_and_return("echo " + str(x + jobOptions["utmEastOffset"]) + " " + str(y + jobOptions["utmNorthOffset"]) + " " + str(z), "gdaltransform -s_srs \"" + jobOptions["csString"] + "\" -t_srs \"EPSG:4326\"")
                            coords_wgs84 = coords_wgs84_out.split(' ')
                            lat_frac = coord_to_fractions(coords_wgs84[1],['N','S'])
                            lon_frac = coord_to_fractions(coords_wgs84[0],['E','W'])

                            exivCmd = "exiv2 -q"
                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitude " + lat_frac[0] + "\""
                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitudeRef " + lat_frac[1] + "\""
                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitude " + lon_frac[0] + "\""
                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitudeRef " + lon_frac[1] + "\""

                            altitude = abs(int(float(coords_wgs84[2])*100))
                            exivCmd += " -M\"set Exif.GPSInfo.GPSAltitude " + str(altitude) + "/100\""
                            exivCmd += " -M\"set Exif.GPSInfo.GPSAltitudeRef "
                            if coords_wgs84[2]>=0:
                                exivCmd += "0"
                            else:
                                exivCmd += "1"
                            exivCmd += "\""
                            exivCmd += " " + filename
                            run(exivCmd)

    if "epsg" in jobOptions and "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions:
        lasCmd = "\"" + BIN_PATH + "/txt2las\" -i " + jobOptions["jobDir"] + "-results/option-0000_georef.ply -o " + jobOptions["jobDir"] + "-results/pointcloud_georef.laz -skip 30 -parse xyzRGBssss -set_scale 0.01 0.01 0.01 -set_offset " + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"]) + " 0 -translate_xyz " + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"]) + " 0 -epsg " + str(jobOptions["epsg"])
        print("    Creating geo-referenced LAS file (expecting warning)...")
        run(lasCmd)


def odm_orthophoto():
    """Run odm_orthophoto"""
    print "\n  - running orthophoto generation - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_orthophoto")
    except:
        pass

    run("\"" + BIN_PATH + "/odm_orthophoto\" -inputFile " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo.obj -logFile " + jobOptions["jobDir"] + "/odm_orthophoto/odm_orthophoto_log.txt -outputFile " + jobOptions["jobDir"] + "-results/odm_orthphoto.png -resolution " + str(args.odm_orthophoto_resolution) + " -outputCornerFile " + jobOptions["jobDir"] + "/odm_orthphoto_corners.txt")

    if "csString" not in jobOptions:
        parse_coordinate_system()

    geoTiffCreated = False
    if ("csString" in jobOptions and
            "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions):
        ulx = uly = lrx = lry = 0.0
        with open(jobOptions["jobDir"] +
                  "/odm_orthphoto_corners.txt") as f:
            for lineNumber, line in enumerate(f):
                if lineNumber == 0:
                    tokens = line.split(' ')
                    if len(tokens) == 4:
                        ulx = float(tokens[0]) + \
                            float(jobOptions["utmEastOffset"])
                        lry = float(tokens[1]) + \
                            float(jobOptions["utmNorthOffset"])
                        lrx = float(tokens[2]) + \
                            float(jobOptions["utmEastOffset"])
                        uly = float(tokens[3]) + \
                            float(jobOptions["utmNorthOffset"])

        print("    Creating GeoTIFF...")
        sys.stdout.write("    ")
        run("gdal_translate -a_ullr " + str(ulx) + " " + str(uly) + " " +
            str(lrx) + " " + str(lry) + " -a_srs \"" + jobOptions["csString"] +
            "\" " + jobOptions["jobDir"] + "-results/odm_orthphoto.png " +
            jobOptions["jobDir"] + "-results/odm_orthphoto.tif")
        geoTiffCreated = True

    if not geoTiffCreated:
        print("    Warning: No geo-referenced orthophoto created due to "
              "missing geo-referencing or corner coordinates.")


if args.use_opensfm:
    sfm_tasks = [
        ("resize", resize),
        ("opensfm", opensfm),
    ]
else:
    sfm_tasks = [
        ("resize", resize),
        ("getKeypoints", getKeypoints),
        ("match", match),
        ("bundler", bundler),
    ]

tasks = sfm_tasks + [
    ("cmvs", cmvs),
    ("pmvs", pmvs),
    ("odm_meshing", odm_meshing),
    ("odm_texturing", odm_texturing),
    ("odm_georeferencing", odm_georeferencing),
    ("odm_orthophoto", odm_orthophoto),
]


if __name__ == '__main__':
    prepare_objects()

    os.chdir(jobOptions["jobDir"])

    if args.run_only is not None:
        args.start_with = args.run_only
        args.end_with = args.run_only

    run_tasks = False
    for name, func in tasks:
        if args.start_with == name:
            run_tasks = True
        if run_tasks:
            func()
        if args.end_with == name:
            break

    if args.zip_results:
        print "\nCompressing results - " + now()
        run("cd " + jobOptions["jobDir"] + "-results/ && tar -czf " +
            jobOptions["jobDir"] + "-results.tar.gz *")

    print "\n  - done - " + now()
