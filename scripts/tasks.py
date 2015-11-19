import log
import system

import opensfm
import datatypes

from resize import resize
from dataset import load_dataset

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

			# setup tasks
			if  task_name == 'load_dataset':
				# setup this task
				command = load_dataset
				inputs = { 'images_dir': _odm_app.images_dir,
				           'args': _odm_app.args ,
		                   'photos': _odm_app.photos }

			elif task_name == 'resize':
				# setup this task
				command = resize
				inputs = { 'images_dir': _odm_app.images_dir,
						   'args': _odm_app.args ,
				           'photos': _odm_app.photos }

			elif task_name == 'opensfm':
				# setup this task
				command = None
				inputs = {}

			elif task_name == 'cmvs':
				# setup this task
				command = None
				inputs = {}

			elif task_name == 'pmvs':
				# setup this task
				command = None
				inputs = {}

			elif task_name == 'odm_meshing':
				# setup this task
				command = None
				inputs = {}

			elif task_name == 'odm_texturing':
				# setup this task
				command = None
				inputs = {}

			elif task_name == 'odm_georeferencing':
				# setup this task
				command = None
				inputs = {}

			elif task_name == 'odm_orthophoto':
				# setup this task
				command = None
				inputs = {}

			elif task_name == 'zip_results':
				# setup this task
				command = None
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
			if task.state == 2:
				log.ODM_INFO('Succeeded task %s: %s - %s' % (task.id, task.name, system.now()))
			else:
				log.ODM_ERROR('Aborted task %s: %s' % (task.id, task.name))


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
		return self.launch_command()

	def launch_command(self):
		if self.command is None:
			log.ODM_ERROR('Call method for task %s not defined' % self.name)
			return 3 # failed
		# run conmmand
		try:
			succeed = self.command(**self.inputs)
			return 2 if succeed else 3 # 2:succeed, 3:failed
		except Exception, e:
			log.ODM_ERROR('Method %s cannot be called' % str(self.command))
			log.ODM_ERROR(str(e))
			return 3 # failed