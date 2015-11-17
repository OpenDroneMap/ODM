#!/usr/bin/python

import context
import system

from datatypes import ODMApp

if __name__ == '__main__':

	print '[INFO] Initializing OpenDroneMap app - %s' % system.now()

	# Initialize odm app
	# internally configure all tasks
	app = ODMApp(context.scripts_path)

	# set from where we want to start
	# by default we will start from the beginnig
	init_task_id = 0

	print '[INFO] Runnning OpenDroneMap app from state %s - %s' % (init_task_id, system.now())

	# run all tasks
	app.run(init_task_id)

 
	print '[INFO] OpenDroneMap app finished - %s' % system.now()