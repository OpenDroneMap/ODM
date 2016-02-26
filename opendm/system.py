import os
import errno
import json
import datetime
import sys
import subprocess

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
    """Make a directory including parent directories.
    """
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
