import log

import resize
import opensfm
import datatypes

from dataset import load_dataset
from resize import resize

# Define pipeline tasks
tasks_dict = { '0': 'load_dataset',
			   '1': 'resize',
               '2': 'opensfm',
               '3': 'cmvs',
               '4': 'pmvs',
               '5': 'odm_meshing',
               '6': 'odm_texturing',
               '7': 'odm_georeferencing',
               '8': 'odm_orthophoto', 
               '9': 'zip_results' }


class ODMTaskManager(object):
	"""docstring for ODMTaskManager"""
	def __init__(self, odm_app):
		self.odm_app = odm_app
		self.initial_task_id = 0
		self.current_task_id = 0
		self.tasks = self.init_tasks(tasks_dict, self.odm_app)

	def init_tasks(self, _tasks_dict, _odm_app):

		# dict to store tasks objects
		tasks = {}

		# loop over tasks dict
		for key, in _tasks_dict:

			# instantiate and append ODMTask
			task_name = _tasks_dict[key]
			tasks[key] = ODMTask(key, task_name)

			# Setup each tasks i/o
			command = None

			if  task_name == 'load_dataset':
				# setup this task
				command = load_dataset
				inputs = { 'images_dir': _odm_app.images_dir,
				           'args': _odm_app.args ,
		                   'photos': _odm_app.photos }

			elif task_name == 'resize':
				# setup this task
				command = resize
				inputs = { 'photos': _odm_app.photos }

			elif task_name == 'opensfm':
				# setup this task
				inputs = {}

			elif task_name == 'cmvs':
				# setup this task
				inputs = {}

			elif task_name == 'pmvs':
				# setup this task
				inputs = {}

			elif task_name == 'odm_meshing':
				# setup this task
				inputs = {}

			elif task_name == 'odm_texturing':
				# setup this task
				inputs = {}

			elif task_name == 'odm_georeferencing':
				# setup this task
				inputs = {}

			elif task_name == 'odm_orthophoto':
				# setup this task
				inputs = {}

			elif task_name == 'zip_results':
				# setup this task
				inputs = {}

			else:
				log.ODM_ERROR('task_name %s is not valid' % task_name)

			# setup task configuration
			task = tasks[key]
			task.command = command
			task.inputs = inputs

		return tasks


	def run_tasks(self):
		for id in range(self.initial_task_id, len(self.tasks)):
			# catch task with current id
			task = self.tasks[str(id)]
			# update task tracking
			log.ODM_INFO('Running task %s: %s' % (task.id, task.name))
			self.current_task_id = task.id
			# run task
			task.state = task.run()


class ODMTask(object):
	"""docstring for ODMTask"""
	def __init__(self, id, name):
		# task definition
		self.id = id
		self.name = name
		# task i/o
		self.command = None
		self.inputs = {}
		# Current task state (0:waiting, 1:running, 2:succeded: 3:failed)
		# By default we set a task in waiting state
		self.state = 0

	# Launch task
	def run(self):
		# while doing something
		self.state = 1
		self.launch_command()
		# if succeeded with current task
		if True:
			self.state = 2
		else:
			self.state = 3
		# Return task state
		return self.state

	def launch_command(self):
		if self.command is None:
			log.ODM_ERROR('Call method for task %s not defined' % self.name)
			return
		# run configured conmmand
		try:
			self.command(**self.inputs)
		except Exception, e:
			log.ODM_ERROR('Method %s cannot be called' % str(self.command))
			log.ODM_ERROR(str(e))
		


		