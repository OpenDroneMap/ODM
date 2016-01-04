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
import csv
import knnMatch_exif

import pprint

# the defs
CURRENT_DIR = os.getcwd()
BIN_PATH_ABS = os.path.abspath(os.path.dirname(os.path.abspath(__file__)))
BIN_PATH = os.path.join(BIN_PATH_ABS, 'bin')
PYOPENCV_PATH = os.path.join(BIN_PATH_ABS, 'lib/python2.7/dist-packages')
OPENSFM_PATH = os.path.join(BIN_PATH_ABS, "src/OpenSfM")
CORES = multiprocessing.cpu_count()


# parse arguments
processopts = ['resize', 'opensfm', 'getKeypoints', 'match', 'bundler', 'cmvs', 'pmvs',
               'odm_meshing', 'odm_texturing', 'odm_georeferencing',
               'odm_orthophoto', 'compress']

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
                    default=6000,
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


def run(cmd):
    """Run a system command"""
    returnCode = os.system(cmd)
    #returnCode = subprocess.call(cmd, shell=True)
    if (returnCode != 0):
        sys.exit("\nquitting, error while runnig command: \n\t" + cmd + "\n\nreturned with code: " + str(returnCode) + "\n")

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



def print_task_header(task):
    """Prints the task to be run"""
    print "\n  - " + str(task) + " - " + datetime.datetime.now().strftime('%a %b %d %H:%M:%S %Z %Y')

def print_task_info(info):
    """Prints sub infos of a task"""
    print "     " + str(info)

def mkdir_p(path):
    """Make a directory including parent directories"""
    try:
        os.makedirs(path)
    except os.error as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise

def coord_to_fractions(coord,refs):
    """Calculates the fraction format of the gps coordinates to be used with exif2"""
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

class ODMPhoto:
    def __init__(self, filename, odmJob):
        filename = filename.rstrip('\n')
        file_exif_info = run_and_return('jhead "' + filename + '"',None)

        self.filename = filename
        self.filenameBase = re.sub("\.[^\.]*$", "", filename)
        self.fullPath = os.path.join(odmJob.srcDir,filename)

        match = re.search("Camera make[\ ]*: ([^\n\r]*)", file_exif_info)
        if match:
            self.make = match.group(1).strip()

        match = re.search("Camera model[\ ]*: ([^\n\r]*)", file_exif_info)
        if match:
            self.model = match.group(1).strip()

        if hasattr(self,'make'):
            self.make = re.sub("^\s+", "", self.make)
            self.make = re.sub("\s+$", "", self.make)

        if hasattr(self,'model'):
            self.model = re.sub("^\s+", "", self.model)
            self.model = re.sub("\s+$", "", self.model)

        if hasattr(self,'make'):
            self.cameraID = self.make
        if hasattr(self,'model'):
            self.cameraID += " " + self.model

        match = re.search("Resolution[\ ]*: ([0-9]*) x ([0-9]*)", file_exif_info)
        if match:
            self.width = int(match.group(1).strip())
            self.height = int(match.group(2).strip())
        if args.force_focal is None:
            match = re.search("Focal length[\ ]*:[\ ]*([0-9\.]*)mm", file_exif_info)
            if match:
                self.focal = float(match.group(1).strip())
        else:
            self.focal = args.force_focal

        if args.force_ccd is None:
            match = re.search("CCD width[\ ]*:[\ ]*([0-9\.]*)mm", file_exif_info)
            if match:
                self.ccd = float(match.group(1).strip())

            if hasattr(self,'ccd')==False and hasattr(self,'cameraID') and self.getCcdWidthForCamera(self.cameraID):
                self.ccd = self.getCcdWidthForCamera(self.cameraID)
        else:
            self.ccd = args.force_ccd

        if hasattr(self,"ccd") and hasattr(self,"focal") and hasattr(self,"width") and hasattr(self,"height"):
            if self.width > self.height:
                self.focalpx = self.width * self.focal / self.ccd
            else:
                self.focalpx = self.height * self.focal / self.ccd
            self.odmJob = odmJob
            self.resizedFullPath = os.path.join(odmJob.jobDir,self.filename)
            self.isOk = True
        else:
            self.isOk = False

    def getCcdWidthForCamera(self,cameraID):
        """Return the CCD Width of the camera listed in the JSON defs file."""
        if hasattr(self,"ccdWidths")==False:
            with open(BIN_PATH_ABS + '/ccd_defs.json') as jsonFile:
                self.ccdWidths = json.load(jsonFile)

        if cameraID in self.ccdWidths:
            return float( self.ccdWidths[cameraID] )
        else:
            return False

    def resize(self):
        if self.isOk:
            if not os.path.isfile(self.resizedFullPath):
                if self.odmJob.resizeTo != 0 and ((int(self.width) > self.odmJob.resizeTo) or (self.height > self.odmJob.resizeTo)):
                    print_task_info("resizing " + self.filename)
                    run("convert -resize " + str(self.odmJob.resizeTo) + "x" + str(self.odmJob.resizeTo) + " -quality 100 \"" + self.fullPath + "\" \"" + self.resizedFullPath + "\"")
                else:
                    print_task_info("copying " + self.filename)
                    shutil.copyfile(self.fullPath, self.resizedFullPath)
            else:
                print_task_info("using existing " + self.filename)

            file_resolution = run_and_return('jhead "' + self.resizedFullPath + '"', 'grep "Resolution"')
            match = re.search(": ([0-9]*) x ([0-9]*)", file_resolution)
            if match:
                self.resizedWidth = int(match.group(1).strip())
                self.resizedHeight = int(match.group(2).strip())

    def fullPathWithOtherExt(self,extension):
        return os.path.join(self.odmJob.jobDir,self.filenameBase+'.'+extension)

class ODMJob:
    def __init__(self, baseFolder):
        print_task_header("configuration")
        for arg in vars(args):
            print_task_info(str(arg) + ": " + str(getattr(args, arg)))

        print_task_header("source files")
        self.srcDir = baseFolder
        self.resizeTo = args.resize_to

        if args.job_dir_name is None:
            args.job_dir_name = "reconstruction-with-image-size-" + str(self.resizeTo)

        self.jobDir = os.path.join(self.srcDir, str(args.job_dir_name))
        self.jobResultsDir = os.path.join(self.srcDir, str(args.job_dir_name)+"-results")

        source_files = run_and_return('ls -1', 'egrep -i "[.]jpe?g$"')

        self.photoStats = {'count': 0, 'good': 0, 'bad': 0}
        self.photos = []

        for filename in source_files.split('\n'):
            filename = filename.rstrip('\n')
            if not filename:
                continue

            photo = ODMPhoto(filename, self)
            if photo.isOk:
                self.photoStats["good"] += 1
                print_task_info("using " + photo.filename + "     dimensions: " + str(photo.width) + "x" + str(photo.height) + " / focal: " + str(photo.focal) + "mm / ccd: " + str(photo.ccd) + "mm")
            else:
                self.photoStats["bad"] += 1
                if hasattr(photo,"cameraID"):
                    print_task_info("no CCD width or focal length found for " + photo.filename + " - camera: \"" + photo.cameraID+ "\"")
                else:
                    print_task_info("no CCD width or focal length found for " + photo.filename)

            self.photoStats["count"] += 1
            self.photos.append( photo )

        if self.photoStats["good"] == 0:
            print_task_info("found no usable images - quitting")
            sys.exit()
        else:
            print_task_info("found " + str(self.photoStats["good"]) + " usable images")

        mkdir_p(self.jobDir)
        os.chdir(self.jobDir)

    def fullPathJobTempFile(self,filename):
        return os.path.join(self.jobDir,filename)

    def writeJobTempFile(self,filename,content):
        f = open(self.fullPathJobTempFile(filename), 'w')
        f.write(content)
        f.close()

    def deleteJobTempFile(self,filename):
        if os.path.isfile(self.fullPathJobTempFile(filename)):
            os.remove(self.fullPathJobTempFile(filename))

    def parseCoordinateSystem(self):
        """reads coordinate system and offset attributes from coord file"""
        if os.path.isfile(self.jobDir + '/odm_georeferencing/coordFile.txt'):
            with open(self.jobDir + '/odm_georeferencing/coordFile.txt') as f:
                for lineNumber, line in enumerate(f):
                    if lineNumber == 0:
                        # check for the WGS84 UTM 17N format
                        match_wgs_utm = re.search('WGS84 UTM (\d{1,2})(N|S)',line.strip(),re.I)
                        if match_wgs_utm:
                            utmZoneString = match_wgs_utm.group(1)
                            utmSouthBool = ( match_wgs_utm.group(2).upper() == 'S' )
                            self.csString = '+datum=WGS84 +proj=utm +zone=' + utmZoneString + (' +south' if utmSouthBool else '')
                            if utmSouthBool:
                                self.epsg = 32700 + utmZoneString
                            else:
                                self.epsg = 32600 + utmZoneString
                        else:
                            self.csString = line.strip()
                    elif lineNumber == 1:
                        tokens = line.split(' ')
                        if len(tokens) == 2:
                            self.eastOffset = int(tokens[0].strip())
                            self.northOffset = int(tokens[1].strip())
                    else:
                        break

    def resize(self):
        """Resize images"""
        print_task_header("resizing images")

        print_task_info("using max image size of " + str(self.resizeTo) + " x " + str(self.resizeTo))

        os.chdir(self.jobDir)

        for photo in self.photos:
            photo.resize()

    def getKeypoints(self):
        """Run vlsift to create keypoint files for each image"""
        print_task_header("finding keypoints")

        os.chdir(self.jobDir)

        vlsiftJobs = ""
        c = 0

        for photo in self.photos:
            c += 1
            if photo.isOk:
                if not os.path.isfile(photo.fullPathWithOtherExt("key.bin")):
                    vlsiftJobs += "echo -n \"     " + str(c).zfill(len(str(self.photoStats["good"]))) + "/" + str(self.photoStats["good"]) + " - \""
                    vlsiftJobs += " && convert -format pgm \"" + photo.resizedFullPath + "\" \"" + photo.fullPathWithOtherExt("pgm") + "\""
                    vlsiftJobs += " && \"" + BIN_PATH + "/vlsift\" \"" + photo.fullPathWithOtherExt("pgm") + "\" -o \"" + photo.fullPathWithOtherExt("key") + ".sift\" > /dev/null"
                    vlsiftJobs += " && perl \"" + BIN_PATH + "/../convert_vlsift_to_lowesift.pl\" \"" + self.jobDir + "/" + photo.filenameBase + "\""
                    vlsiftJobs += " && gzip -f \"" + photo.fullPathWithOtherExt("key") + "\""
                    vlsiftJobs += " && rm -f \"" + photo.fullPathWithOtherExt("pgm") + "\""
                    vlsiftJobs += " && rm -f \"" + photo.fullPathWithOtherExt("key") + ".sift\""
                    vlsiftJobs += " && echo \"\"\n"
                else:
                    print_task_info("using existing " + photo.fullPathWithOtherExt("key.bin"))

        if vlsiftJobs != "":
            self.writeJobTempFile('_vlsift.templist.txt',vlsiftJobs)
            run("\"" + BIN_PATH + "/parallel\" --no-notice --halt-on-error 1 -j+0 < \"" + self.fullPathJobTempFile('_vlsift.templist.txt') + "\"")
            self.deleteJobTempFile('_vlsift.templist.txt')

    def match(self):
        """Run matches on images"""
        print_task_header("matching keypoints")

        os.chdir(self.jobDir)
        mkdir_p(self.fullPathJobTempFile('matches'))

        matchesJobs = ""
        c = 0

        preselected_pairs = []

        # Create a file list with all keypoint files
        filesList = ""
        for photo in self.photos:
            if photo.isOk:
                filesList += photo.fullPathWithOtherExt("key") + "\n"
        self.writeJobTempFile('_filelist.templist.txt',filesList)

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
            preselected_pairs = knnMatch_exif.preselect_pairs(BIN_PATH + "/odm_extract_utm", self.fullPathJobTempFile('_filelist.templist.txt'), k_distance, use_knn_mode)

        if len(preselected_pairs) != 0:
            # preselection was succesfull
            for i, j, in preselected_pairs:
                c += 1
                if i < 10:
                    print i, j
                if not os.path.isfile(self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt"):
                    matchesJobs += "echo -n \".\" && touch \"" + self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt\" && \"" + BIN_PATH + "/KeyMatch\" \"" + self.photos[i].fullPathWithOtherExt("key") + "\" \"" + self.photos[j].fullPathWithOtherExt("key") + "\" \"" + self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt\" " + str(args.matcher_ratio) + " " + str(args.matcher_threshold) + "\n"
        else:
            # preselection failed, Match all image pairs
            if do_preselection:
                print_task_info("Failed to run pair preselection, proceeding with exhaustive matching.")
            for i in range(0, self.photoStats["good"]):
                for j in range(i + 1, self.photoStats["good"]):
                    c += 1
                    if not os.path.isfile(self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt"):
                        matchesJobs += "echo -n \".\" && touch \"" + self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt\" && \"" + BIN_PATH + "/KeyMatch\" \"" + self.photos[i].fullPathWithOtherExt("key") + "\" \"" + self.photos[j].fullPathWithOtherExt("key") + "\" \"" + self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt\" " + str(args.matcher_ratio) + " " + str(args.matcher_threshold) + "\n"

        self.writeJobTempFile('_matches_jobs.templist.txt',matchesJobs)
        run("\"" + BIN_PATH + "/parallel\" --no-notice --halt-on-error 1 -j+0 < \"" + self.fullPathJobTempFile('_matches_jobs.templist.txt') + "\"")
        self.deleteJobTempFile('_matches_jobs.templist.txt')

        # TODO: Why?
        run("rm -f \"" + self.fullPathJobTempFile('matches.init.txt') + "\"")

        for i in range(0, self.photoStats["good"]):
            for j in range(i + 1, self.photoStats["good"]):
                c += 1
                if os.path.isfile(self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt") and os.path.getsize(self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt") > 0:
                    run("echo \"" + str(i) + " " + str(j) + "\" >> \"" + self.fullPathJobTempFile('matches.init.txt') + "\" && cat \"" + self.fullPathJobTempFile('matches') + "/" + str(i) + "-" + str(j) + ".txt\" >> \"" + self.fullPathJobTempFile('matches.init.txt') + "\"")


    #   run("\"" + BIN_PATH + "/KeyMatchFull\" \"" + self.fullPathJobTempFile('_filelist.templist.txt') + "\" \"" + self.fullPathJobTempFile('matches.init.txt') + "\"    ")
        self.deleteJobTempFile('_filelist.templist.txt')

    def bundler(self):
        """Run bundler and prepare bundle for PMVS"""
        print_task_header("running bundler")

        os.chdir(self.jobDir)

        mkdir_p(self.jobDir + "/bundle")
        mkdir_p(self.jobDir + "/pmvs")
        mkdir_p(self.jobDir + "/pmvs/txt")
        mkdir_p(self.jobDir + "/pmvs/visualize")
        mkdir_p(self.jobDir + "/pmvs/models")

        filesList = ""
        for photo in self.photos:
            if photo.isOk:
                filesList += "./" + photo.filename + " 0 {:.5f}\n".format(photo.focalpx)

        filesList = filesList.rstrip('\n')
        self.writeJobTempFile('list.txt',filesList)

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
        self.writeJobTempFile('options.txt',bundlerOptions)


        run("\"" + BIN_PATH + "/bundler\" \"" + self.fullPathJobTempFile('list.txt') + "\" --options_file \"" + self.fullPathJobTempFile('options.txt') + "\" > bundle/out")
        print_task_info("running Bundle2PMVS")
        run("\"" + BIN_PATH + "/Bundle2PMVS\" \"" + self.fullPathJobTempFile('list.txt') + "\" bundle/bundle.out")
        print_task_info("running RadialUndistort")
        run("\"" + BIN_PATH + "/RadialUndistort\" \"" + self.fullPathJobTempFile('list.txt') + "\" bundle/bundle.out pmvs")

        i = 0
        for photo in self.photos:
            if photo.isOk:
                if os.path.isfile("pmvs/" + photo.filenameBase + ".rd.jpg"):
                    nr = "{0:08d}".format(i)
                    i += 1

                    run("mv pmvs/" + photo.filenameBase + ".rd.jpg pmvs/visualize/" + str(nr) + ".jpg")
                    run("mv pmvs/" + str(nr) + ".txt pmvs/txt/" + str(nr) + ".txt")

        print_task_info("running Bundle2Vis")
        run("\"" + BIN_PATH + "/Bundle2Vis\" pmvs/bundle.rd.out pmvs/vis.dat")


    def opensfm(self):
        print_task_header("running OpenSfM")

        os.chdir(self.jobDir)

        # Create bundler's list.txt
        filesList = ""
        for photo in self.photos:
            if photo.isOk:
                filesList += "./" + photo.filename + " 0 {:.5f}\n".format(photo.focalpx)

        filesList = filesList.rstrip('\n')
        self.writeJobTempFile('list.txt',filesList)

        # Create opensfm working folder
        mkdir_p("opensfm")

        # Configure OpenSfM
        config = [
           "use_exif_size: no",
           "feature_process_size: {}".format(self.resizeTo),
           "feature_min_frames: {}".format(args.min_num_features),
           "processes: {}".format(CORES),
           "matching_gps_neighbors: {}".format(args.matcher_neighbors),
        ]

        if args.matcher_distance>0:
            config.append("matching_gps_distance: {}".format(args.matcher_distance))

        self.writeJobTempFile('opensfm/config.yaml',"\n".join(config))

        # Convert bundler's input to opensfm
        print_task_info("running import_bundler")
        run('PYTHONPATH={} "{}/bin/import_bundler" opensfm --list list.txt'.format(PYOPENCV_PATH, OPENSFM_PATH))

        # Run OpenSfM reconstruction
        print_task_info("running OpenSfM reconstruction")
        run('PYTHONPATH={} "{}/bin/run_all" opensfm'.format(PYOPENCV_PATH, OPENSFM_PATH))

        # Convert back to bundler's format
        print_task_info("running export_bundler")
        run('PYTHONPATH={} "{}/bin/export_bundler" opensfm'.format(PYOPENCV_PATH, OPENSFM_PATH))

        self.bundler_to_pmvs("opensfm/bundle_r000.out")



    def bundler_to_pmvs(self,bundle_out):
        """Converts bundler's output to PMVS format"""
        print_task_header("converting bundler output to PMVS")

        os.chdir(self.jobDir)

        mkdir_p(self.jobDir + "/pmvs")
        mkdir_p(self.jobDir + "/pmvs/txt")
        mkdir_p(self.jobDir + "/pmvs/visualize")
        mkdir_p(self.jobDir + "/pmvs/models")

        print_task_info("running Bundle2PMVS")
        run("\"" + BIN_PATH + "/Bundle2PMVS\" \"" + self.fullPathJobTempFile('list.txt') + "\" " + bundle_out)
        print_task_info("running RadialUndistort")
        run("\"" + BIN_PATH + "/RadialUndistort\" \"" + self.fullPathJobTempFile('list.txt') + "\" " + bundle_out + " pmvs")

        i = 0
        for photo in self.photos:
            if photo.isOk:
                if os.path.isfile("pmvs/" + photo.filenameBase + ".rd.jpg"):
                    nr = "{0:08d}".format(i)
                    i += 1

                    run("mv pmvs/" + photo.filenameBase + ".rd.jpg pmvs/visualize/" + str(nr) + ".jpg")
                    run("mv pmvs/" + str(nr) + ".txt pmvs/txt/" + str(nr) + ".txt")

        print_task_info("running Bundle2Vis")
        run("\"" + BIN_PATH + "/Bundle2Vis\" pmvs/bundle.rd.out pmvs/vis.dat")


    def cmvs(self):
        """Run CMVS"""
        print_task_header("running cmvs")

        os.chdir(self.jobDir)

        run("\"" + BIN_PATH + "/cmvs\" pmvs/ " + str(args.cmvs_maxImages) + " " + str(CORES))
        run("\"" + BIN_PATH + "/genOption\" pmvs/ " + str(args.pmvs_level) + " " + str(args.pmvs_csize) + " " + str(args.pmvs_threshold) + " " + str(args.pmvs_wsize) + " " + str(args.pmvs_minImageNum) + " " + str(CORES))


    def pmvs(self):
        """Run PMVS"""
        print_task_header("running pmvs")

        os.chdir(self.jobDir)

        run("\"" + BIN_PATH + "/pmvs2\" pmvs/ option-0000")

        run("cp -Rf \"" + self.jobDir + "/pmvs/models\" \"" + jobResultsDir + "\"")


    def odm_meshing(self):
        """Run odm_meshing"""
        print_task_header("running meshing")

        os.chdir(self.jobDir)
        mkdir_p(self.jobDir + "/odm_meshing")

        run("\"" + BIN_PATH + "/odm_meshing\" -inputFile " + self.jobResultsDir + "/option-0000.ply -outputFile " + jobResultsDir + "/odm_mesh-0000.ply -logFile " + self.jobDir + "/odm_meshing/odm_meshing_log.txt -maxVertexCount " + str(args.odm_meshing_maxVertexCount) + " -octreeDepth " + str(args.odm_meshing_octreeDepth) + " -samplesPerNode " + str(args.odm_meshing_samplesPerNode) + " -solverDivide " + str(args.odm_meshing_solverDivide))


    def odm_texturing(self):
        """Run odm_texturing"""
        print_task_header("running texturing")

        os.chdir(self.jobDir)
        mkdir_p(self.jobDir + "/odm_texturing")
        mkdir_p(self.jobResultsDir + "/odm_texturing")

        # Create list of original image files
        pmvs_list = self.jobDir + "/pmvs/list.rd.txt"
        texturing_list = self.jobDir + "/odm_texturing/image_list.txt"
        with open(pmvs_list) as fin:
            with open(texturing_list, "w") as fout:
                for line in fin:
                    base = line.rstrip('\n')[2:-4]
                    for photo in self.photos:
                        if photo.filenameBase == base:
                            fout.write("./{}\n".format(photo.filename))
                            break

        run("\"" + BIN_PATH + "/odm_texturing\" -bundleFile " + self.jobDir + "/pmvs/bundle.rd.out -imagesPath " + self.srcDir + "/ -imagesListPath " + texturing_list + " -inputModelPath " + self.jobResultsDir + "/odm_mesh-0000.ply -outputFolder " + self.jobResultsDir + "/odm_texturing/ -textureResolution " + str(args.odm_texturing_textureResolution) + " -bundleResizedTo " + str(self.resizeTo) + " -textureWithSize " + str(args.odm_texturing_textureWithSize) + " -logFile " + self.jobDir + "/odm_texturing/odm_texturing_log.txt")


    def odm_georeferencing(self):
        """Run odm_georeferencing"""
        print_task_header("running georeferencing")

        os.chdir(self.jobDir)
        mkdir_p(self.jobDir + "/odm_georeferencing")

        if not args.odm_georeferencing_useGcp:
            run("\"" + BIN_PATH + "/odm_extract_utm\" -imagesPath " + self.srcDir + "/ -imageListFile " + self.jobDir + "/pmvs/list.rd.txt -outputCoordFile " + self.jobDir + "/odm_georeferencing/coordFile.txt")
            run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + self.jobDir + "/pmvs/bundle.rd.out -inputCoordFile " + self.jobDir + "/odm_georeferencing/coordFile.txt -inputFile " + self.jobResultsDir + "/odm_texturing/odm_textured_model.obj -outputFile " + self.jobResultsDir + "/odm_texturing/odm_textured_model_geo.obj -inputPointCloudFile " + self.jobResultsDir + "/option-0000.ply -outputPointCloudFile " + self.jobResultsDir + "/option-0000_georef.ply -logFile " + self.jobDir + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + self.jobResultsDir + "/odm_texturing/odm_textured_model_geo_georef_system.txt")
        elif os.path.isfile(self.srcDir + "/" + args.odm_georeferencing_gcpFile):
            run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + self.jobDir + "/pmvs/bundle.rd.out -gcpFile " + self.srcDir + "/" + args.odm_georeferencing_gcpFile + " -imagesPath " + self.srcDir + "/ -imagesListPath " + self.jobDir + "/pmvs/list.rd.txt -bundleResizedTo " + str(self.resizeTo) + " -inputFile " + self.jobResultsDir + "/odm_texturing/odm_textured_model.obj -outputFile " + self.jobResultsDir + "/odm_texturing/odm_textured_model_geo.obj -outputCoordFile " + self.jobDir + "/odm_georeferencing/coordFile.txt -inputPointCloudFile " + self.jobResultsDir + "/option-0000.ply -outputPointCloudFile " + self.jobResultsDir + "/option-0000_georef.ply -logFile " + self.jobDir + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + self.jobResultsDir + "/odm_texturing/odm_textured_model_geo_georef_system.txt")
        else:
            print_task_info("Warning: No GCP file.")
            print_task_info("Skipping orthophoto")
            args.end_with = "odm_georeferencing"

        if hasattr(self,"csString") == False:
            self.parseCoordinateSystem()

        if hasattr(self,"csString") and hasattr(self,"eastOffset") and hasattr(self,"northOffset"):
            images = []
            with open(self.jobDir + "/pmvs/list.rd.txt") as f:
                images = f.readlines()

            if len(images) > 0:
                print_task_info("Writing EXIF of photo taking GPS coordinates to resized images...")
                with open(self.jobDir + "/odm_georeferencing/coordFile.txt") as f:
                    for lineNumber, line in enumerate(f):
                        if lineNumber >= 2 and lineNumber - 2 < len(images):
                            tokens = line.split(' ')
                            if len(tokens) >= 3:
                                x = float(tokens[0])
                                y = float(tokens[1])
                                z = float(tokens[2])
                                filename = images[lineNumber - 2]
                                coords_wgs84_out = run_and_return("echo " + str(x + self.eastOffset) + " " + str(y + self.northOffset) + " " + str(z), "gdaltransform -s_srs \"" + self.csString + "\" -t_srs \"EPSG:4326\"")
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

        if hasattr(self,"epsg") and hasattr(self,"eastOffset") and hasattr(self,"northOffset"):
            lasCmd = "\"" + BIN_PATH + "/txt2las\" -i " + self.jobResultsDir + "/option-0000_georef.ply -o " + self.jobResultsDir + "/pointcloud_georef.laz -skip 30 -parse xyzRGBssss -set_scale 0.01 0.01 0.01 -set_offset " + str(self.eastOffset) + " " + str(self.northOffset) + " 0 -translate_xyz " + str(self.eastOffset) + " " + str(self.northOffset) + " 0 -epsg " + str(self.epsg)
            print_task_info("Creating geo-referenced LAS file (expecting warning)...")
            run(lasCmd)

        # XYZ point cloud output
        print_task_info("Creating geo-referenced CSV file (XYZ format, can be used with GRASS to create DEM)...")
        with open(self.jobResultsDir + "/pointcloud_georef.csv", "wb") as csvfile:
            csvfile_writer = csv.writer(csvfile, delimiter=",")
            reachedPoints = False
            with open(self.jobResultsDir + "/option-0000_georef.ply") as f:
                for lineNumber, line in enumerate(f):
                    if reachedPoints:
                        tokens = line.split(" ")
                        csv_line = [float(tokens[0])+self.eastOffset,float(tokens[1])+self.northOffset,tokens[2]]
                        csvfile_writer.writerow(csv_line)
                    if line.startswith("end_header"):
                        reachedPoints = True
            csvfile.close()


    def odm_orthophoto(self):
        """Run odm_orthophoto"""
        print_task_header("running orthophoto generation")

        os.chdir(self.jobDir)
        mkdir_p(self.jobDir + "/odm_orthophoto")

        run("\"" + BIN_PATH + "/odm_orthophoto\" -inputFile " + self.jobResultsDir + "/odm_texturing/odm_textured_model_geo.obj -logFile " + self.jobDir + "/odm_orthophoto/odm_orthophoto_log.txt -outputFile " + self.jobResultsDir + "/odm_orthphoto.png -resolution " + str(args.odm_orthophoto_resolution) + " -outputCornerFile " + self.jobDir + "/odm_orthphoto_corners.txt")

        if hasattr(self,"csString") == False:
            self.parseCoordinateSystem()

        geoTiffCreated = False
        if (hasattr(self,"csString") and hasattr(self,"eastOffset") and hasattr(self,"northOffset")):
            ulx = uly = lrx = lry = 0.0
            with open(self.jobDir + "/odm_orthphoto_corners.txt") as f:
                for lineNumber, line in enumerate(f):
                    if lineNumber == 0:
                        tokens = line.split(' ')
                        if len(tokens) == 4:
                            ulx = float(tokens[0]) + \
                                float(self.eastOffset)
                            lry = float(tokens[1]) + \
                                float(self.northOffset)
                            lrx = float(tokens[2]) + \
                                float(self.eastOffset)
                            uly = float(tokens[3]) + \
                                float(self.northOffset)

            print_task_info("Creating GeoTIFF...")
            run("gdal_translate -a_ullr " + str(ulx) + " " + str(uly) + " " +
                str(lrx) + " " + str(lry) + " -a_srs \"" + self.csString +
                "\" " + self.jobResultsDir + "/odm_orthphoto.png " +
                self.jobResultsDir + "/odm_orthphoto.tif")
            geoTiffCreated = True

        if not geoTiffCreated:
            print_task_info("Warning: No geo-referenced orthophoto created due to missing geo-referencing or corner coordinates.")

    def compress_results(self):
        """Compresses the final results folder"""
        print_task_header("compressing results")
        run("cd " + self.jobResultsDir + " && tar -czf " + self.jobResultsDir + ".tar.gz *")




if __name__ == '__main__':

    args = parser.parse_args()

    odmJob = ODMJob(CURRENT_DIR)


    if args.use_opensfm:
        sfm_tasks = [
            ("resize", "resize"),
            ("opensfm", "opensfm"),
        ]
    else:
        sfm_tasks = [
            ("resize", "resize"),
            ("getKeypoints", "getKeypoints"),
            ("match", "match"),
            ("bundler", "bundler"),
        ]

    tasks = sfm_tasks + [
        ("cmvs", "cmvs"),
        ("pmvs", "pmvs"),
        ("odm_meshing", "odm_meshing"),
        ("odm_texturing", "odm_texturing"),
        ("odm_georeferencing", "odm_georeferencing"),
        ("odm_orthophoto", "odm_orthophoto"),
        ("compress", "compress_results")
    ]

    if args.run_only is not None:
        args.start_with = args.run_only
        args.end_with = args.run_only

    run_tasks = False
    for name, func in tasks:
        if args.start_with == name:
            run_tasks = True
        if run_tasks:
            eval("odmJob."+func+"()")
        if args.end_with == name:
            break

    print_task_header("done")
