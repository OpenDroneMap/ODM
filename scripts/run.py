#!/usr/bin/python

import sys

import log
import config
import system

from datatypes import ODMApp

def usage():
	log.ODM_ERROR('USAGE: %s --images-src dataset_path' % sys.argv[0])
 	log.ODM_ERROR('OpenDroneMap app finished - %s' % system.now())
 	sys.exit(0)
		
if __name__ == '__main__':

	log.ODM_INFO('Initializing OpenDroneMap app - %s' % system.now())

	# Force to provide the images path
	if config.args.get('images_src') is None:
		usage()

	# Initialize odm app
	# internally configure all tasks
	app = ODMApp(config.args)

	# set from where we want to start
	# by default we will start from the beginnig
	init_task_id = 0

	log.ODM_INFO('Runnning OpenDroneMap app - %s' % system.now())

	# run all tasks
	app.run_all(init_task_id)

 	log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())