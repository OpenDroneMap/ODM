#!/usr/bin/python
import sys
import os
import json

BIN_PATH_ABS = os.path.abspath(os.path.dirname(os.path.abspath(__file__)))

def get_ccd_widths():
	"""Return the CCD Width of the camera listed in the JSON defs file."""
	with open(BIN_PATH_ABS + '/ccd_defs.json') as jsonFile:
		return json.load(jsonFile)

try:
	ccd_defs = get_ccd_widths()
	print "CCD_DEFS compiles OK"
	print "Definitions in file: {0}".format(len(ccd_defs))
	exit_code=0
except IOError as e:
	print "I/O error with CCD_DEFS file: {0}".format(e.strerror)
	exit_code=255
except:
	print "Error with CCD_DEFS file: {0}".format(sys.exc_info()[1])
	exit_code=255

sys.exit(exit_code)
