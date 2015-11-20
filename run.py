#!/usr/bin/python

import sys

from opendm import log
from opendm import config
from opendm import system
from opendm.datatypes import ODMApp

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

	log.ODM_INFO('Runnning OpenDroneMap app - %s' % system.now())

	# run single task
	if config.args.get('run_only') is not None:
		# task to run
		tasks = config.args['run_only']
		app.run_all(tasks, tasks)

	# run multiple tasks
	else:
		# get initial and final tasks
		initial_task = config.args['start_with']
		final_task = config.args['end_with']

		# run tasks
		app.run_all(initial_task, final_task)

 	log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())