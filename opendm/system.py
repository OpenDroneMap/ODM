import os
import errno
import json
import datetime
import sys
import subprocess
import string

from opendm import context
from opendm import log


def get_ccd_widths():
    """Return the CCD Width of the camera listed in the JSON defs file."""
    with open(context.ccd_widths_path) as f:
        sensor_data = json.loads(f.read())
    return dict(zip(map(string.lower, sensor_data.keys()), sensor_data.values()))


def run(cmd):
    """Run a system command"""
    log.ODM_DEBUG('running %s' % cmd)
    retcode = subprocess.call(cmd, shell=True)

    if retcode < 0:
        raise Exception("Child was terminated by signal {}".format(-retcode))
    elif retcode > 0:
        raise Exception("Child returned {}".format(retcode))


def now():
    """Return the current time"""
    return datetime.datetime.now().strftime('%a %b %d %H:%M:%S %Z %Y')


def now_raw():
    return datetime.datetime.now()


def benchmark(start, benchmarking_file, process):
    """
    runs a benchmark with a start datetime object
    :return: the running time (delta)
    """
    # Write to benchmark file
    delta = (datetime.datetime.now() - start).total_seconds()
    with open(benchmarking_file, 'a') as b:
        b.write('%s runtime: %s seconds\n' % (process, delta))


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
