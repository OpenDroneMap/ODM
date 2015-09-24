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
import fractions
import argparse
import errno
<<<<<<< HEAD

import knnMatch_exif
=======
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

import knnMatch_exif

<<<<<<< HEAD
# # #
# 0 - the defs
# # #

=======
# the defs
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
CURRENT_DIR = os.getcwd()
BIN_PATH_ABS = os.path.abspath(os.path.dirname(os.path.abspath(__file__)))
PYOPENCV_PATH = os.path.join(BIN_PATH_ABS, 'lib/python2.7/dist-packages')
OPENSFM_PATH = os.path.join(BIN_PATH_ABS, "src/OpenSfM")
CORES = multiprocessing.cpu_count()

<<<<<<< HEAD
objects = []

BIN_PATH = BIN_PATH_ABS + os.sep +"bin"


# # #
# 1 - generic functions
# # #

def extractFloat(inputString):
    x = map(float, re.findall(r'[+-]?[0-9.]+', inputString))
    return x

def get_ccd_widths():
    """Return the CCD Width of the camera listed in the JSON defs file."""
    with open(BIN_PATH_ABS + '/ccd_defs.json') as jsonFile:
        return json.load(jsonFile)

def remove_values_from_list(the_list, val):
    '''  Remove the <val> from <the_list> '''
    return [value for value in the_list if value != val]




#->Q:  get_ccd_widths prolly belongs somewhere else

# #
# 1 - Classes
# #

class ODMJob:
    '''   ODMJob - a class for ODM Activities
    '''
    def __init__(self, inputDir, args):
        self.args = args
        self.pathDirJPGs = inputDir
        
        self.count    = 0
        self.good     = 0
        self.bad      = 0
        self.minWidth = 0.
        self.minHeight= 0
        self.maxWidth = 0
        self.maxHeight= 0
        
        self.resizeTo = 0.
        self.srcDir = CURRENT_DIR
        self.utmZone = -999
        self.utmSouth = False
        self.utmEastOffset = 0.
        self.utmNorthOffset = 0.
        
        self.jobOptions = {'resizeTo': 0, 'srcDir': CURRENT_DIR, 'utmZone': -999, 
                      'utmSouth': False, 'utmEastOffset': 0, 'utmNorthOffset': 0}
        
        self.dictJobLocations = {}   # hold our filepaths and dirs

        self.listFiles = os.listdir(CURRENT_DIR)
        self.listJPG = []
        self.listObjPhotos = []    

        
        # create obj.listJPG of all jpegs 
        for files in self.listFiles:
            (pathfn,ext) = os.path.splitext(files)
            if 'jpg' in ext.lower() :
                #print files
                self.listJPG.append(files)
            elif 'jpeg' in ext.lower() : 
                self.listJPG.append(files)
                
        print "\n  - source files - " + now()
    
        for filename in self.listJPG:
            filename = filename.rstrip('\n')
            if not filename:
                continue
            filename = CURRENT_DIR + os.sep + filename 
            print filename
            self.listObjPhotos.append( ODMPhoto( filename, self) )        

    def resize(self):
        print "\n  - preparing images - " + now()
    
        os.chdir(self.jobDir)
    
        for objPhotos in self.listObjPhotos:
            if objPhotos.isOk:
                if not os.path.isfile(objPhotos.dictStepVals["step_0_resizedImage"]):
                    if self.resizeTo != 0 and \
                       ((int(objPhotos.width) > self.resizeTo) or (objPhotos.height > self.resizeTo)):
                       
                        sys.stdout.write("    resizing " + objPhotos.strFileName +" \tto " \
                                         + objPhotos.dictStepVals["step_0_resizedImage"])
                        
                        run("convert -resize " + str(self.resizeTo) + "x" + str(self.resizeTo)\
                            +" -quality 100 \"" + self.srcDir + "/" + objPhotos.strFileName + "\" \"" + objPhotos.dictStepVals["step_0_resizedImage"] + "\"")
                    
                    else:
                        sys.stdout.write("     copying " + objPhotos.strFileName + " \tto " + objPhotos.dictStepVals["step_0_resizedImage"])
                        shutil.copyfile(CURRENT_DIR + "/" + objPhotos.strFileName, objPhotos.dictStepVals["step_0_resizedImage"])
                else:	
                    print "     using existing " + objPhotos.strFileName + " \tto " + objPhotos.dictStepVals["step_0_resizedImage"]
    
                file_resolution = runAndReturn('jhead "' + objPhotos.dictStepVals["step_0_resizedImage"] + '"', 'grep "Resolution"')
                match = re.search(": ([0-9]*) x ([0-9]*)", file_resolution)
                if match:
                    objPhotos.width = int(match.group(1).strip())
                    objPhotos.height = int(match.group(2).strip())
                print "\t (" + str(objPhotos.width) + " x " + str(objPhotos.height) + ")"
    
    def getKeypoints(self):
        print "\n  - finding keypoints - " + now()
    
        os.chdir(self.jobDir)
    
        vlsiftJobs = ""
        c = 0
    
        for objPhotos in self.listObjPhotos:
            c += 1
    
            if objPhotos.isOk:
                if '--lowe-sift' in self.args:
                    vlsiftJobs    += "echo -n \"" + str(c) + "/" + str(self.good)     \
                        + " - \" && convert -format pgm \"" + self.dictJobLocations["step_0_resizedImage"] \
                        + "\" \"" + self.dictJobLocations["step_1_pgmFile"] + "\""
                    
                    vlsiftJobs    += " && \"" + BIN_PATH + "/sift\" < \"" + self.dictJobLocations["step_1_pgmFile"] \
                        + "\" > \"" + self.dictJobLocations["step_1_keyFile"] + "\""
                    
                    vlsiftJobs    += " && gzip -f \"" + self.dictJobLocations["step_1_keyFile"] + "\""

                    vlsiftJobs    += " && rm -f \"" + self.dictJobLocations["step_1_pgmFile"] + "\""

                    vlsiftJobs    += " && rm -f \"" + self.dictJobLocations["step_1_keyFile"] + ".sift\"\n"
               
                else:
                    if not os.path.isfile(self.jobDir + "/" +objPhotos.strFileNameBase + ".key.bin"):
                        vlsiftJobs    += "echo -n \"" + str(c) + "/" + str(self.good) +  \
                            " - \" && convert -format pgm \"" + self.dictJobLocations["step_0_resizedImage"] +\
                            "\" \"" + self.dictJobLocations["step_1_pgmFile"] + "\""
                        
                        vlsiftJobs    += " && \"" + BIN_PATH + "/vlsift\" \"" + self.dictJobLocations["step_1_pgmFile"] \
                            + "\" -o \"" + self.dictJobLocations["step_1_keyFile"] + ".sift\" > /dev/null && perl \""   \
                            + BIN_PATH + "/../convert_vlsift_to_lowesift.pl\" \"" + self.jobDir     \
                            + "/" +objPhotos.strFileNameBase + "\""
                        
                        vlsiftJobs    += " && gzip -f \"" + self.dictJobLocations["step_1_keyFile"] + "\""
                        
                        vlsiftJobs    += " && rm -f \"" + self.dictJobLocations["step_1_pgmFile"] + "\""
                        
                        vlsiftJobs    += " && rm -f \"" + self.dictJobLocations["step_1_keyFile"] + ".sift\"\n"
                        
                    else:
                        print "using existing " + self.jobDir + "/" +objPhotos.strFileNameBase + ".key.bin"
    
        siftDest = open(self.dictJobLocations["step_1_vlsift"], 'w')
        siftDest.write(vlsiftJobs)
        siftDest.close()
    
        run("\"" + BIN_PATH + "/parallel\" --no-notice --halt-on-error 1 -j+0 < \"" + self.dictJobLocations["step_1_vlsift"] + "\"")
        
    def match():
        print "\n  - matching keypoints - " + now()
    
        os.chdir(self.jobDir)
        try:
            os.mkdir(self.dictJobLocations["step_2_matches_dir"])
        except:
            pass
    
        matchesJobs = ""   
        c = 0
        t = (self.good - 1) * (self.good / 2)    
    
        preselected_pairs = []
    
        # Create a file list with all keypoint files
        filesList = ""
        for objPhotos in self.listObjPhotos:
            if objPhotos.isOk:
                filesList += objPhotos.dictStepVals["step_1_keyFile"] + "\n"
        matchDest = open(self.dictJobLocations["step_2_filelist"], 'w')
        matchDest.write(filesList)
        matchDest.close()
    
        #Check if preselection is to be run
        if self.args["matcher_preselect"]:
            useKnn = True
            if self.args["matcher_useKnn"]:
                useKnn = False
            preselected_pairs = knnMatch_exif.preselect_pairs(BIN_PATH + os.sep +"odm_extract_utm", 
                                                              self.dictJobLocations["step_2_filelist"], 
                                                              self.args["matcher_kDistance"], 
                                                              self.args["matcher_useKnn"])
        
        if len(preselected_pairs) != 0:
            for i, j, in preselected_pairs:
                c += 1
                if i < 10:
                    print i, j            
                if not os.path.isfile(jobOptions["step_2_matches_dir"] +\
                                      "/" + str(i) + "-" + str(j) + ".txt"):
                    
                    matchesJobs += "echo -n \".\" && touch \"" + self.dictJobLocations["step_2_matches_dir"]     \
                        + "/" + str(i) + "-" + str(j) + ".txt\" && \"" + BIN_PATH + "/KeyMatch\" \""             \
                        + self.listObjPhotos[i].dictStepVals["step_1_keyFile"] + "\" \""                         \
                        + self.listObjPhotos[j].dictStepVals["step_1_keyFile"]                                   \
                        + "\" \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j)               \
                        + ".txt\" " + str(args['--matcher-ratio']) + " " + str(args['--matcher-threshold']) + "\n"
    
    
    # Match all image pairs
        else:
            if args["--matcher-preselect"] == "true":
                print "Failed to run pair preselection, proceeding with exhaustive matching."
            for i in range(0, objectStats["good"]):
                for j in range(i + 1, objectStats["good"]):
                    c += 1
                    if not os.path.isfile(jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt"):
                        matchesJobs += "echo -n \".\" && touch \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" && \"" + BIN_PATH + "/KeyMatch\" \"" + objects[i]["step_1_keyFile"] + "\" \"" + objects[j]["step_1_keyFile"] + "\" \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" " + str(args['--matcher-ratio']) + " " + str(args['--matcher-threshold']) + "\n"
    
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
        
class ODMPhoto:
    """   ODMPhoto - a class for ODMPhotos
    """

    def __init__(self, inputJPG, objODMJob):
        #general purpose
        verbose = False
        # object attributes
        self.dictStepVals           = {}
        self.pathToContainingFolder = os.path.split(inputJPG)[0]
        self.strFileName            = os.path.split(inputJPG)[1]
        self.strFileNameBase        = os.path.splitext(self.strFileName)[0]
        self.strFileNameExt         = os.path.splitext(self.strFileName)[1]
        
        #start pipe for jhead
        cmdSrc = BIN_PATH + os.sep + "jhead "+ CURRENT_DIR + os.sep + self.strFileName
        srcProcess = subprocess.Popen(cmdSrc, stdout=subprocess.PIPE)

        stdout, stderr = srcProcess.communicate()
        stringOutput = stdout.decode('ascii')
        
        #listOutput is the list of params to be processed
        listOutput_ori = stringOutput.splitlines()
        listOutput = remove_values_from_list(listOutput_ori,u"")
        
        intListCount = 0
        intNumCameraAtts = len(listOutput)
        
        flagDoneList = False
        
        if verbose:  print listOutput
        
        for lines in listOutput:
            # check if we've read all atts
            intListCount += 1
            if intListCount == intNumCameraAtts: flagDoneList = True
           
            #extract and proceed            
            firstColon = lines.find(":")
            tempKey =   lines[:firstColon].strip()
            tempVal = lines[firstColon+1:].strip()
            
            if verbose:   print tempKey,tempVal
            # all them values
            if tempKey == 'File name': self.fileName = tempVal
            elif tempKey == 'File size': self.fileSize= tempVal
            elif tempKey == 'File date': self.fileDate = tempVal
            elif tempKey == 'Camera make': self.cameraMake = tempVal
            elif tempKey == 'Camera model': self.cameraModel = tempVal
            elif tempKey == 'Date/Time': self.dateTime = tempVal
            elif tempKey == 'Resolution': self.resolution = tempVal
            elif tempKey == 'Flash used': self.flashUsed = tempVal
            elif tempKey == 'Focal length': self.focalLength = tempVal
            elif tempKey == 'CCD width': self.ccdWidth = tempVal
            elif tempKey == 'Exposure time': self.exposureTime = tempVal
            elif tempKey == 'Aperture': self.aperture = tempVal
            elif tempKey == 'Focus dist.': self.focusDist = tempVal
            elif tempKey == 'ISO equiv.': self.isoEquiv= tempVal
            elif tempKey == 'Whitebalance': self.whitebalance = tempVal
            elif tempKey == 'Metering Mode': self.meteringMode = tempVal
            elif tempKey == 'GPS Latitude': self.gpsLatitude = tempVal
            elif tempKey == 'GPS Longitude': self.gpsLongitude = tempVal
            elif tempKey == 'GPS Altitude': self.gpsAltitude = tempVal
            elif tempKey == 'JPEG Quality': self.jpgQuality = tempVal	
            #  better object attribute names; keep old for compatability
            #  shallow references point to same stack space
            self.fullPathAndName = self.fileName

            # attribute 'id' set to more specific of the maker or model
            try:
                if self.cameraMake:
                    self.make = self.cameraMake
                    self.id = self.cameraMake
            except:  pass

            
            try:
                if self.cameraModel:
                    self.model = self.cameraModel
                    self.id = self.cameraModel
            except:  pass
            
            # parse resolution field 
            try:
                match = re.search("([0-9]*) x ([0-9]*)",self.resolution)
                if match:
                    self.width  = int(match.group(1).strip())
                    self.height = int(match.group(2).strip())
            except:  pass
            
            #parse force-focal
            try:
                if not '--force-focal' in args:
                    match = re.search(":[\ ]*([0-9\.]*)mm", self.focalLength)
                    if match:
                        self.focal = float((match.group()[1:-2]).strip())
                else:
                    self.focal = args['--force-focal']
            except: pass

            #parse force-ccd
            if 'ccd' in lines.lower():
                if not '--force-ccd' in args:
                    try:
                        floats = extractFloat(self.ccdWidth)
                        self.ccd = floats[0]
                    except:
                        try:
                            self.ccd = float(ccdWidths[self.id])
                        except: pass
                else:
                    self.ccd = args['--force-ccd']


            if verbose:  print intListCount
            
            if flagDoneList:
                try:
                    if self.width > self.height:
                        self.focalpx = self.width * (self.focal / self.ccd)
                    else:
                        self.focalpx = self.height * (self.focal / self.ccd)
        
                    self.isOk = True
                    objODMJob.good += 1
        
                    print "     using " + self.fileName + "     dimensions: " + \
                          str(self.width) + "x" + str(self.height)\
                          + " | focal: " + str(self.focal) \
                          + "mm | ccd: " + str(self.ccd) + "mm"
                    
                except:
                    self.isOk = False
                    objODMJob.bad += 1
    
                    try:
                        print "\n    no CCD width or focal length found for "\
                              + self.fileName+ " - camera: \"" + self.id+ "\""
                    except:
                        print "\n    no CCD width or focal length found"
            
                #either way increment total count
                objODMJob.count += 1
    
                #populate & update max/mins
                
                if objODMJob.minWidth == 0:
                    objODMJob.minWidth = self.width
                               
                if objODMJob.minHeight == 0:
                    objODMJob.minHeight = self.height

                if objODMJob.minWidth < self.width:
                    objODMJob.minWidth = self.minWidth
                else:
                    objODMJob.minWidth = self.width
               
                if objODMJob.minHeight < self.height:
                    objODMJob.minHeight = objODMJob.minHeight
                else:
                    objODMJob.minHeight = self.height

                if objODMJob.maxWidth > self.width:
                    objODMJob.maxWidth = objODMJob.maxWidth
                else:
                    objODMJob.maxWidth = self.width

                if objODMJob.maxHeight > self.height:
                    objODMJob.maxHeight = objODMJob.maxHeight
                else:
                    objODMJob.maxHeight = self.height
    


# #
# 2 - Parse Arguments
# #
=======
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
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
processopts = ['resize', 'getKeypoints', 'match', 'bundler', 'cmvs', 'pmvs',
               'odm_meshing', 'odm_texturing', 'odm_georeferencing',
               'odm_orthophoto']

parser = argparse.ArgumentParser(description='OpenDroneMap')
parser.add_argument('--resize-to',  # currently doesn't support 'orig'
                    metavar='<integer>',
                    default=2400,
                    type=int,
                    help='resizes images by the largest side')

parser.add_argument('--start-with', '-s',
                    metavar='<string>',
                    default='resize',
                    choices=processopts,
                    help=('Can be one of: ' + ' | '.join(processopts)))

parser.add_argument('--end-with', '-e',
                    metavar='<string>',
                    default='odm_orthophoto',
                    choices=processopts,
                    help=('Can be one of:' + ' | '.join(processopts)))

parser.add_argument('--run-only',
                    metavar='<string>',
                    choices=processopts,
                    help=('Can be one of:' + ' | '.join(processopts)))

parser.add_argument('--force-focal',
                    metavar='<positive float>',
                    type=float,
                    help=('Override the focal length information for the '
<<<<<<< HEAD
                          'images'))
=======
                            'images'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--force-ccd',
                    metavar='<positive float>',
                    type=float,
                    help='Override the ccd width information for the images')

parser.add_argument('--matcher-threshold',
                    metavar='<percent>',
                    default=2.0,
                    type=float,
                    help=('Ignore matched keypoints if the two images share '
<<<<<<< HEAD
                          'less than <float> percent of keypoints'))
=======
                            'less than <float> percent of keypoints'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--matcher-ratio',
                    metavar='<float>',
                    default=0.6,
                    type=float,
                    help=('Ratio of the distance to the next best matched '
<<<<<<< HEAD
                          'keypoint'))
=======
                            'keypoint'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--matcher-preselect',
                    type=bool,
                    metavar='',
                    default=True,
                    help=('use GPS exif data, if available, to match each '
<<<<<<< HEAD
                          'image only with its k-nearest neighbors, or all '
                          'images within a certain distance threshold'))
=======
                            'image only with its k-nearest neighbors, or all '
                            'images within a certain distance threshold'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--matcher-useKnn',
                    type=bool,
                    metavar='',
                    default=True,
                    help=('use GPS exif data, if available, to match each '
<<<<<<< HEAD
                          'image only with its k-nearest neighbors, or all '
                          'images within a certain distance threshold'))
=======
                            'image only with its k-nearest neighbors, or all '
                            'images within a certain distance threshold'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--matcher-kDistance',
                    metavar='<integer>',
                    default=20,
                    type=int,
                    help='')

parser.add_argument('--cmvs-maxImages',
                    metavar='<integer>',
                    default=500,
                    type=int,
                    help='The maximum number of images per cluster')

parser.add_argument('--pmvs-level',
                    metavar='<positive integer>',
                    default=1,
                    type=int,
                    help=('The level in the image pyramid that is used '
<<<<<<< HEAD
                          'for the computation. see '
                          'http://www.di.ens.fr/pmvs/documentation.html for '
                          'more pmvs documentation'))
=======
                            'for the computation. see '
                            'http://www.di.ens.fr/pmvs/documentation.html for '
                            'more pmvs documentation'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--pmvs-csize',
                    metavar='< positive integer>',
                    default=2,
                    type=int,
                    help='Cell size controls the density of reconstructions')

parser.add_argument('--pmvs-threshold',
                    metavar='<float: -1.0 <= x <= 1.0>',
                    default=0.7,
                    type=float,
                    help=('A patch reconstruction is accepted as a success '
<<<<<<< HEAD
                          'and kept, if its associcated photometric consistency '
                          'measure is above this threshold.'))
=======
                            'and kept, if its associcated photometric consistency '
                            'measure is above this threshold.'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--pmvs-wsize',
                    metavar='<positive integer>',
                    default=7,
                    type=int,
                    help=('pmvs samples wsize x wsize pixel colors from '
<<<<<<< HEAD
                          'each image to compute photometric consistency '
                          'score. For example, when wsize=7, 7x7=49 pixel '
                          'colors are sampled in each image. Increasing the '
                          'value leads to more stable reconstructions, but '
                          'the program becomes slower.'))
=======
                            'each image to compute photometric consistency '
                            'score. For example, when wsize=7, 7x7=49 pixel '
                            'colors are sampled in each image. Increasing the '
                            'value leads to more stable reconstructions, but '
                            'the program becomes slower.'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--pmvs-minImageNum',
                    metavar='<positive integer>',
                    default=3,
                    type=int,
                    help=('Each 3D point must be visible in at least '
<<<<<<< HEAD
                          'minImageNum images for being reconstructed. 3 is '
                          'suggested in general.'))
=======
                            'minImageNum images for being reconstructed. 3 is '
                            'suggested in general.'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--odm_meshing-maxVertexCount',
                    metavar='<positive integer>',
                    default=100000,
                    type=int,
                    help='The maximum vertex count of the output mesh')

parser.add_argument('--odm_meshing-octreeDepth',
                    metavar='<positive integer>',
                    default=9,
                    type=int,
                    help=('Oct-tree depth used in the mesh reconstruction, '
<<<<<<< HEAD
                          'increase to get more vertices, recommended '
                          'values are 8-12'))
=======
                            'increase to get more vertices, recommended '
                            'values are 8-12'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--odm_meshing-samplesPerNode',
                    metavar='<float >= 1.0>',
                    default=1,
                    type=float,
                    help=('Number of points per octree node, recommended '
<<<<<<< HEAD
                          'value: 1.0'))
=======
                            'value: 1.0'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--odm_meshing-solverDivide',
                    metavar='<positive integer>',
                    default=9,
                    type=int,
                    help=('Oct-tree depth at which the Laplacian equation '
<<<<<<< HEAD
                          'is solved in the surface reconstruction step. '
                          'Increasing this value increases computation '
                          'times slightly but helps reduce memory usage.'))
=======
                            'is solved in the surface reconstruction step. '
                            'Increasing this value increases computation '
                            'times slightly but helps reduce memory usage.'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--odm_texturing-textureResolution',
                    metavar='<positive integer>',
                    default=4096,
                    type=int,
                    help=('The resolution of the output textures. Must be '
<<<<<<< HEAD
                          'greater than textureWithSize.'))
=======
                            'greater than textureWithSize.'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--odm_texturing-textureWithSize',
                    metavar='<positive integer>',
                    default=3600,
                    type=int,
                    help=('The resolution to rescale the images performing '
<<<<<<< HEAD
                          'the texturing.'))
=======
                            'the texturing.'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--odm_georeferencing-gcpFile',
                    metavar='<path string>',
                    default='gcp_list.txt',
                    help=('path to the file containing the ground control '
<<<<<<< HEAD
                          'points used for georeferencing.The file needs to '
                          'be on the following line format: \neasting '
                          'northing height pixelrow pixelcol imagename'))
=======
                            'points used for georeferencing.The file needs to '
                            'be on the following line format: \neasting '
                            'northing height pixelrow pixelcol imagename'))
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

parser.add_argument('--odm_georeferencing-useGcp',
                    type = bool,
                    default = False,
                    help = 'set to true for enabling GCPs from the file above')

parser.add_argument('--zip-results',
                    action='store_true',
                    default=False,
                    help='compress the results using gunzip')

parser.add_argument('--use-opensfm',
                    type=bool,
                    default=False,
                    help='use OpenSfM instead of Bundler to find the camera positions '
<<<<<<< HEAD
                    '(replaces getKeypoints, match and bundler steps)')

args = parser.parse_args()

print "\n               - configuration:              "
#  Print parsed values
#print vars(args)
for key in args.__dict__.keys():
    prString = str(key).ljust(32," ") + "| " + str(args.__dict__[key]).ljust(40," ")
    print prString
print "\n"
# # # 
# 3 - Mainline Functions (refactor into methods)
# # #
=======
                         '(replaces getKeypoints, match and bundler steps)')

args = parser.parse_args()

print "\n  - configuration:"

print vars(args)
# for key in vars(args):
#     if key != "":
#         print "    " + key + ": " + str(args[key])
# print "\n"


>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
def run(cmd):
    """Run a system command"""
    print 'running', cmd
    returnCode = os.system(cmd)
    print 'b'
    if (returnCode != 0):
        sys.exit("\nquitting cause: \n\t" + cmd + "\nreturned with code " +
                 str(returnCode) + ".\n")
<<<<<<< HEAD

def now():
    """Return the current time"""
    return datetime.datetime.now().strftime('%a %b %d %H:%M:%S %Z %Y')


=======


def now():
    """Return the current time"""
    return datetime.datetime.now().strftime('%a %b %d %H:%M:%S %Z %Y')


>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
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
                    tokens = line.split(' ')
                    if len(tokens) == 3:
                        utmZoneString = tokens[2][0:len(tokens[2])-2].strip()
                        utmSouthBool = (tokens[2][len(tokens[2])-2].strip() == 'S')
                        jobOptions['csString'] = '+datum=WGS84 +proj=utm +zone=' + utmZoneString + (' +south' if utmSouthBool else '')
                        jobOptions['epsg'] = calculate_EPSG(int(utmZoneString), utmSouthBool)
                elif lineNumber == 1:
                    tokens = line.split(' ')
                    if len(tokens) == 2:
                        jobOptions['utmEastOffset'] = int(tokens[0].strip())
                        jobOptions['utmNorthOffset'] = int(tokens[1].strip())
                else:
                    break

<<<<<<< HEAD

# # # 
# 3b is this really part of main?
# # # 
def prepare_objects():
    ## get the source list

    
    objODMJob = ODMJob(CURRENT_DIR, args)

    if not objODMJob.good > 0:
        print "\n    found no usable images - quitting\n"
        sys.exit()
    else:
        print "\n    found " + str(objODMJob.good) + " usable images"

    print "\n"
=======

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

    jobOptions["jobDir"] = jobOptions["srcDir"] + "/reconstruction-with-image-size-" + str(jobOptions["resizeTo"])

    jobOptions["step_1_convert"] = jobOptions["jobDir"] + "/_convert.templist.txt"
    jobOptions["step_1_vlsift"] = jobOptions["jobDir"] + "/_vlsift.templist.txt"
    jobOptions["step_1_gzip"] = jobOptions["jobDir"] + "/_gzip.templist.txt"

    jobOptions["step_2_filelist"] = jobOptions["jobDir"] + "/_filelist.templist.txt"
    jobOptions["step_2_macthes_jobs"] = jobOptions["jobDir"] + "/_matches_jobs.templist.txt"
    jobOptions["step_2_matches_dir"] = jobOptions["jobDir"] + "/matches"
    jobOptions["step_2_matches"] = jobOptions["jobDir"] + "/matches.init.txt"

    jobOptions["step_3_filelist"] = jobOptions["jobDir"] + "/list.txt"
    jobOptions["step_3_bundlerOptions"] = jobOptions["jobDir"] + "/options.txt"
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

    objODMJob.resizeTo = args.resize_to

<<<<<<< HEAD
    print "  using max image size of " + str( objODMJob.resizeTo) + " x " + str(objODMJob.resizeTo)

    objODMJob.jobDir = objODMJob.srcDir + "//reconstruction-with-image-size-" + str( objODMJob.resizeTo)
=======
    for fileObject in objects:
        if fileObject["isOk"]:
            fileObject["step_0_resizedImage"] = jobOptions["jobDir"] + "/" + fileObject["src"]
            fileObject["step_1_pgmFile"] = jobOptions["jobDir"] + "/" + fileObject["base"] + ".pgm"
            fileObject["step_1_keyFile"] = jobOptions["jobDir"] + "/" + fileObject["base"] + ".key"
            fileObject["step_1_gzFile"] = jobOptions["jobDir"] + "/" + fileObject["base"] + ".key.gz"


def resize():
    """Resize images"""
    print "\n  - preparing images - " + now()
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

    objODMJob.dictJobLocations["step_1_convert"]        = objODMJob.jobDir + "/_convert.templist.txt"
    objODMJob.dictJobLocations["step_1_vlsift"]         = objODMJob.jobDir + "/_vlsift.templist.txt"
    objODMJob.dictJobLocations["step_1_gzip"]           = objODMJob.jobDir + "/_gzip.templist.txt"

<<<<<<< HEAD
    objODMJob.dictJobLocations["step_2_filelist"]       = objODMJob.jobDir + "/_filelist.templist.txt"
    objODMJob.dictJobLocations["step_2_macthes_jobs"]   = objODMJob.jobDir + "/_matches_jobs.templist.txt"
    objODMJob.dictJobLocations["step_2_matches_dir"]    = objODMJob.jobDir + "/matches"
    objODMJob.dictJobLocations["step_2_matches"]        = objODMJob.jobDir + "/matches.init.txt"

    objODMJob.dictJobLocations["step_3_filelist"]       = objODMJob.jobDir + "/list.txt"
    objODMJob.dictJobLocations["step_3_bundlerOptions"] = objODMJob.jobDir + "/options.txt"

=======
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
                vlsiftJobs += "echo -n \"" + str(c) + "/" + str(objectStats["good"]) + " - \" && convert -format pgm \"" + fileObject["step_0_resizedImage"] + "\" \"" + fileObject["step_1_pgmFile"] + "\""
                vlsiftJobs += " && \"" + BIN_PATH + "/vlsift\" \"" + fileObject["step_1_pgmFile"] + "\" -o \"" + fileObject["step_1_keyFile"] + ".sift\" > /dev/null && perl \"" + BIN_PATH + "/../convert_vlsift_to_lowesift.pl\" \"" + jobOptions["jobDir"] + "/" + fileObject["base"] + "\""
                vlsiftJobs += " && gzip -f \"" + fileObject["step_1_keyFile"] + "\""
                vlsiftJobs += " && rm -f \"" + fileObject["step_1_pgmFile"] + "\""
                vlsiftJobs += " && rm -f \"" + fileObject["step_1_keyFile"] + ".sift\"\n"
            else:
                print "using existing " + jobOptions["jobDir"] + "/" + fileObject["base"] + ".key.bin"

    siftDest = open(jobOptions["step_1_vlsift"], 'w')
    siftDest.write(vlsiftJobs)
    siftDest.close()

    run("\"" + BIN_PATH + "/parallel\" --no-notice --halt-on-error 1 -j+0 < \"" + jobOptions["step_1_vlsift"] + "\"")


def match():
    """Run matches on images"""
    print "\n  - matching keypoints - " + now()

    os.chdir(jobOptions["jobDir"])
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
    try:
        os.mkdir(objODMJob.jobDir)
    except:
        pass
<<<<<<< HEAD

    for objPhotos in objODMJob.listObjPhotos:
        if objPhotos.isOk: 
            objPhotos.dictStepVals["step_0_resizedImage"] = objODMJob.jobDir +\
                os.sep + str(objPhotos.strFileNameBase) + ".jpg"
            
            objPhotos.dictStepVals["step_1_pgmFile"]      = objODMJob.jobDir +\
                os.sep + str(objPhotos.strFileNameBase) + ".pgm"
            
            objPhotos.dictStepVals["step_1_keyFile"]      = objODMJob.jobDir +\
                os.sep + str(objPhotos.strFileNameBase) + ".key"
            
            objPhotos.dictStepVals["step_1_gzFile"]       = objODMJob.jobDir +\
                os.sep + str(objPhotos.strFileNameBase) + ".key.gz"
    return objODMJob

=======

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

    # Check if preselection is to be run
    if args.matcher_preselect:
        useKnn = True
        if args.matcher_useKnn:
            useKnn = False  # BUG: never used
        preselected_pairs = knnMatch_exif.preselect_pairs(BIN_PATH + "/odm_extract_utm", jobOptions["step_2_filelist"], args.matcher_kDistance, args.matcher_useKnn)
    if len(preselected_pairs) != 0:
        for i, j, in preselected_pairs:
            c += 1
            if i < 10:
                print i, j
            if not os.path.isfile(jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt"):
                matchesJobs += "echo -n \".\" && touch \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" && \"" + BIN_PATH + "/KeyMatch\" \"" + objects[i]["step_1_keyFile"] + "\" \"" + objects[j]["step_1_keyFile"] + "\" \"" + jobOptions["step_2_matches_dir"] + "/" + str(i) + "-" + str(j) + ".txt\" " + str(args.matcher_ratio) + " " + str(args.matcher_threshold) + "\n"


# Match all image pairs
    else:
        if args.matcher_preselect == "true":
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

>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

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
<<<<<<< HEAD
            filesList += "./" + fileObject["base"] + ".jpg 0 {:.5f}\n".format(fileObject["focalpx"])

    filesList = filesList.rstrip('\n')

    bundlerOptions  = "--match_table matches.init.txt\n"
=======
            filesList += "./" + fileObject["src"] + " 0 {:.5f}\n".format(fileObject["focalpx"])

    filesList = filesList.rstrip('\n')

    bundlerOptions = "--match_table matches.init.txt\n"
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
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
<<<<<<< HEAD
                i += 1                

                run("mv pmvs/" + fileObject["base"] + ".rd.jpg pmvs/visualize/" + str(nr) + ".jpg")                
                run("mv pmvs/" + str(nr) + ".txt pmvs/txt/" + str(nr) + ".txt")                

    run("\"" + BIN_PATH + "/Bundle2Vis\" pmvs/bundle.rd.out pmvs/vis.dat")                

    if args['--end-with'] != "bundler":        
=======
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
       "features_process_size: {}".format(jobOptions["resizeTo"]),
       "preemptive_threshold: 5",
       "processes: {}".format(CORES),
    ]
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

    if args.end_with != "bundler":
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
        cmvs()


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

<<<<<<< HEAD
    run("\"" + BIN_PATH + "/cmvs\" pmvs/ " + str(args['--cmvs-maxImages']) + " " + str(CORES))
    run("\"" + BIN_PATH + "/genOption\" pmvs/ " + str(args['--pmvs-level']) + " " + str(args['--pmvs-csize']) + " " + str(args['--pmvs-threshold']) + " " + str(args['--pmvs-wsize']) + " " + str(args['--pmvs-minImageNum']) + " " + str(CORES))

    if args['--end-with'] != "cmvs":
        pmvs()
=======
    run("\"" + BIN_PATH + "/cmvs\" pmvs/ " + str(args.cmvs_maxImages) + " " + str(CORES))
    run("\"" + BIN_PATH + "/genOption\" pmvs/ " + str(args.pmvs_level) + " " + str(args.pmvs_csize) + " " + str(args.pmvs_threshold) + " " + str(args.pmvs_wsize) + " " + str(args.pmvs_minImageNum) + " " + str(CORES))

>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

def pmvs():
    """Run PMVS"""
    print "\n  - running pmvs - " + now()

    os.chdir(jobOptions["jobDir"])

    run("\"" + BIN_PATH + "/pmvs2\" pmvs/ option-0000")

    run("cp -Rf \"" + jobOptions["jobDir"] + "/pmvs/models\" \"" + jobOptions["jobDir"] + "-results\"")

<<<<<<< HEAD
    if args['--end-with'] != "pmvs":
        odm_meshing()
=======
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

def odm_meshing():
    """Run odm_meshing"""
    print "\n  - running meshing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_meshing")
    except:
        pass

<<<<<<< HEAD
    run("\"" + BIN_PATH + "/odm_meshing\" -inputFile " + jobOptions["jobDir"] + "-results/option-0000.ply -outputFile " + jobOptions["jobDir"] + "-results/odm_mesh-0000.ply -logFile " + jobOptions["jobDir"] + "/odm_meshing/odm_meshing_log.txt -maxVertexCount " + str(args['--odm_meshing-maxVertexCount']) + " -octreeDepth " + str(args['--odm_meshing-octreeDepth']) + " -samplesPerNode " + str(args['--odm_meshing-samplesPerNode']) + " -solverDivide " + str(args['--odm_meshing-solverDivide']))

    if args['--end-with'] != "odm_meshing":
        odm_texturing()
=======
    run("\"" + BIN_PATH + "/odm_meshing\" -inputFile " + jobOptions["jobDir"] + "-results/option-0000.ply -outputFile " + jobOptions["jobDir"] + "-results/odm_mesh-0000.ply -logFile " + jobOptions["jobDir"] + "/odm_meshing/odm_meshing_log.txt -maxVertexCount " + str(args.odm_meshing_maxVertexCount) + " -octreeDepth " + str(args.odm_meshing_octreeDepth) + " -samplesPerNode " + str(args.odm_meshing_samplesPerNode) + " -solverDivide " + str(args.odm_meshing_solverDivide))

>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

def odm_texturing():
    """Run odm_texturing"""
    print "\n  - running texturing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_texturing")
        os.mkdir(jobOptions["jobDir"] + "-results/odm_texturing")
    except:
        pass

<<<<<<< HEAD
    run("\"" + BIN_PATH + "/odm_texturing\" -bundleFile " + jobOptions["jobDir"] + "/pmvs/bundle.rd.out -imagesPath "+ jobOptions["srcDir"] + "/ -imagesListPath " + jobOptions["jobDir"] + "/pmvs/list.rd.txt -inputModelPath " + jobOptions["jobDir"] + "-results/odm_mesh-0000.ply -outputFolder " + jobOptions["jobDir"] + "-results/odm_texturing/ -textureResolution " + str(args['--odm_texturing-textureResolution']) + " -bundleResizedTo " + str(jobOptions["resizeTo"]) + " -textureWithSize " + str(args['--odm_texturing-textureWithSize']) + " -logFile " + jobOptions["jobDir"] + "/odm_texturing/odm_texturing_log.txt")

    if args['--end-with'] != "odm_texturing":
        odm_georeferencing()
=======
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

>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

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
                            run("echo " + str(x + jobOptions["utmEastOffset"]) + " " + str(y + jobOptions["utmNorthOffset"]) + " " + str(z) + " | cs2cs " + jobOptions["csString"] + " +to +datum=WGS84 +proj=latlong > " + jobOptions["jobDir"] + "/odm_georeferencing/latlong.txt")
                            with open(jobOptions["jobDir"] + "/odm_georeferencing/latlong.txt") as latlongFile:
                                latlongLine = latlongFile.readline()
                                tokens = latlongLine.split()
                                if len(tokens) >= 2:
                                    exifGpsInfoWritten = False

                                    lonString = tokens[0]  # Example: 83d18'16.285"W
                                    latString = tokens[1]  # Example: 41d2'11.789"N
                                    altString = ""
                                    if len(tokens) > 2:
<<<<<<< HEAD
                                        altString = tokens[2] # Example: 0.998
=======
                                        altString = tokens[2]  # Example: 0.998
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

                                    tokens = re.split("[d '\"]+", lonString)
                                    if len(tokens) >= 4:
                                        lonDeg = tokens[0]
                                        lonMin = tokens[1]
                                        lonSec = tokens[2]
                                        lonSecFrac = fractions.Fraction(lonSec)
                                        lonSecNumerator = str(lonSecFrac._numerator)
                                        lonSecDenominator = str(lonSecFrac._denominator)
                                        lonRef = tokens[3]

                                        tokens = re.split("[d '\"]+", latString)
                                        if len(tokens) >= 4:
                                            latDeg = tokens[0]
                                            latMin = tokens[1]
                                            latSec = tokens[2]
                                            latSecFrac = fractions.Fraction(latSec)
                                            latSecNumerator = str(latSecFrac._numerator)
                                            latSecDenominator = str(latSecFrac._denominator)
                                            latRef = tokens[3]

                                            exivCmd = "exiv2 -q"
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitude " + latDeg + "/1 " + latMin + "/1 " + latSecNumerator + "/" + latSecDenominator + "\""
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitudeRef " + latRef + "\""
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitude " + lonDeg + "/1 " + lonMin + "/1 " + lonSecNumerator + "/" + lonSecDenominator + "\""
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitudeRef " + lonRef + "\""

                                            altNumerator = arcDenominator = 0  # BUG: arcDenominator is never used
                                            if altString:
                                                altFrac = fractions.Fraction(altString)
                                                altNumerator = str(altFrac._numerator)
                                                altDenominator = str(altFrac._denominator)
                                                exivCmd += " -M\"set Exif.GPSInfo.GPSAltitude " + altNumerator + "/" + altDenominator + "\""
                                                exivCmd += " -M\"set Exif.GPSInfo.GPSAltitudeRef 0\""

                                            exivCmd += " " + filename
                                            run(exivCmd)
                                            exifGpsInfoWritten = True

                                    if not exifGpsInfoWritten:
                                        print("    Warning: Failed setting EXIF GPS info for " + filename + " based on " + latlongLine)

    if "epsg" in jobOptions and "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions:
        lasCmd = "\"" + BIN_PATH + "/txt2las\" -i " + jobOptions["jobDir"] + "-results/option-0000_georef.ply -o " + jobOptions["jobDir"] + "-results/pointcloud_georef.laz -skip 30 -parse xyzRGBssss -set_scale 0.01 0.01 0.01 -set_offset " + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"]) + " 0 -translate_xyz " + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"]) + " 0 -epsg " + str(jobOptions["epsg"])
        print("    Creating geo-referenced LAS file (expecting warning)...")
        run(lasCmd)
<<<<<<< HEAD

    if args['--end-with'] != "odm_georeferencing":
        odm_orthophoto()
=======
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468


def odm_orthophoto():
    """Run odm_orthophoto"""
    print "\n  - running orthophoto generation - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_orthophoto")
    except:
        pass

    run("\"" + BIN_PATH + "/odm_orthophoto\" -inputFile " + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo.obj -logFile " + jobOptions["jobDir"] + "/odm_orthophoto/odm_orthophoto_log.txt -outputFile " + jobOptions["jobDir"] + "-results/odm_orthphoto.png -resolution 20.0 -outputCornerFile " + jobOptions["jobDir"] + "/odm_orthphoto_corners.txt")

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
<<<<<<< HEAD
        print "    Warning: No geo-referenced orthophoto created due to missing geo-referencing or corner coordinates."



#if args['--start-with'] == "resize":
    #resize()
#elif args['--start-with'] == "getKeypoints":
    #getKeypoints()
#elif args['--start-with'] == "match":
    #match()
#elif args['--start-with'] == "bundler":
    #bundler()
#elif args['--start-with'] == "cmvs":
    #cmvs()
#elif args['--start-with'] == "pmvs":
    #pmvs()
#elif args['--start-with'] == "odm_meshing":
    #odm_meshing()
#elif args['--start-with'] == "odm_texturing":
    #odm_texturing()
#elif args['--start-with'] == "odm_georeferencing":
    #odm_georeferencing()
#elif args['--start-with'] == "odm_orthophoto":
    #odm_orthophoto()

#if args['--zip-results'] == True: 
    #print "\nCompressing results - " + now()
    #run("cd " + jobOptions["jobDir"] + "-results/ && tar -czf " + jobOptions["jobDir"] + "-results.tar.gz *")
    
if __name__ == '__main__':
    
    objODMJob = prepare_objects()

    os.chdir(objODMJob.jobDir)

    dictSteps = {0:"resize", 1:"getKeypoints", 2:"match",
                 3:"bundler", 4:"cmvs", 5:"pmvs", 6:"odm_meshing",
                 7:"odm_texturing", 8:"odm_georeferencing",
                 9:"odm_orthophoto", 10:"zip_results"}
        
    listJobQueue = []
    intStart = -1
    intEnd = -2
    
    for keys in dictSteps.keys():
        if dictSteps[keys] == objODMJob.args.start_with:
            intStart = keys
        if dictSteps[keys] == objODMJob.args.end_with:
            intEnd = keys
    if intStart > intEnd:
        sys.stdout.writelines("No Valid Steps - Exitting.")
        exit(0)
    
    for i in range(intStart,intEnd,1):
        listJobQueue.append(dictSteps[i])
    
    for steps in listJobQueue:
        methodCall = steps+"()"
        strEval = "objODMJob."+methodCall
        eval(strEval)
=======
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

    run_tasks = False
    for name, func in tasks:
        if args.start_with == name:
            run_tasks = True
        if run_tasks:
            func()
        if args.end_with == name:
            break
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468

    if args.zip_results:
        print "\nCompressing results - " + now()
        run("cd " + jobOptions["jobDir"] + "-results/ && tar -czf " +
            jobOptions["jobDir"] + "-results.tar.gz *")

<<<<<<< HEAD
    print "\n  - done - " + now()
=======
    print "\n  - done - " + now()
>>>>>>> 2c83a0bdb17e881dd99f66035f76c147598a5468
