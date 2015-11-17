#!/usr/bin/python

import sys

import system
import log

from datatypes import ODMApp

def usage():
	log.ODM_WARNING('USAGE: %s dataset_path' % sys.argv[0])
 	log.ODM_ERROR('OpenDroneMap app finished - %s' % system.now())
 	sys.exit(0)
		
if __name__ == '__main__':

	log.ODM_INFO('Initializing OpenDroneMap app - %s' % system.now())

	if len(sys.argv) < 2:
		usage()
	else:
		dataset_path = sys.argv[1]

	# Initialize odm app
	# internally configure all tasks
	app = ODMApp(dataset_path)

	# set from where we want to start
	# by default we will start from the beginnig
	init_task_id = 0

	log.ODM_INFO('Runnning OpenDroneMap app from state %s - %s' % (init_task_id, system.now()))

	# run all tasks
	app.run(init_task_id)

 	log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())