import os
import errno
import json
import datetime
import sys

from opendm import context
from opendm import log

def get_ccd_widths():
    """Return the CCD Width of the camera listed in the JSON defs file."""
    with open(context.ccd_widths_path) as jsonFile:
        return json.load(jsonFile)

def run(cmd):
    """Run a system command"""
    log.ODM_DEBUG('running %s' % cmd)
    returnCode = os.system(cmd)

    if (returnCode != 0):
        # TODO(edgar): add as log.ODM_ERROR
        sys.exit("\nquitting cause: \n\t" + cmd + "\nreturned with code " +
                 str(returnCode) + ".\n")

def now():
    """Return the current time"""
    return datetime.datetime.now().strftime('%a %b %d %H:%M:%S %Z %Y')

def run_and_return(cmdSrc, cmdDest=None):
    """Run a system command and return the output"""
    process = subprocess.Popen(cmdSrc, stdout=subprocess.PIPE, shell=True)
    stdout, stderr = process.communicate()
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
                        jobOptions['csString'] = '+datum=WGS84 +proj=utm +zone='               \
                            + utmZoneString + (' +south' if utmSouthBool else '')
                        jobOptions['epsg'] = calculate_EPSG(int(utmZoneString), utmSouthBool)
                elif lineNumber == 1:
                    tokens = line.split(' ')
                    if len(tokens) == 2:
                        jobOptions['utmEastOffset'] = int(tokens[0].strip())
                        jobOptions['utmNorthOffset'] = int(tokens[1].strip())
                else:
                    break
