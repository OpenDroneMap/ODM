import log

import resize
import opensfm

# Define pipeline tasks
tasks_dict = { '0': 'resize',
               '1': 'opensfm',
               '2': 'cmvs',
               '3': 'pmvs',
               '4': 'odm_meshing',
               '5': 'odm_texturing',
               '6': 'odm_georeferencing',
               '7': 'odm_orthophoto', 
               '8': 'zip_results' }


class ODMTaskManager(object):
	"""docstring for ODMTaskManager"""
	def __init__(self):
		self.initial_task_id = 0
		self.current_task_id = 0
		self.tasks = self.init_tasks(tasks_dict)

	def init_tasks(self, _tasks_dict):

		# dict to store tasks objects
		tasks = {}

		# loop over tasks dict
		for key, in _tasks_dict:

			# instantiate and append ODMTask
			task_name = _tasks_dict[key]
			tasks[key] = ODMTask(key, task_name)

			# Setup each tasks i/o
			command = None

			if task_name == 'resize':
				# setup this task
				num_inputs = 1
				num_outputs = 1
				command = task_name + '.' + task_name
				inputs = {}
				outputs = {}

			elif task_name == 'opensfm':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}

			elif task_name == 'cmvs':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}

			elif task_name == 'pmvs':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}

			elif task_name == 'odm_meshing':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}

			elif task_name == 'odm_texturing':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}

			elif task_name == 'odm_georeferencing':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}

			elif task_name == 'odm_orthophoto':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}

			elif task_name == 'zip_results':
				# setup this task
				num_inputs = 0
				num_outputs = 0
				inputs = {}
				outputs = {}
			else:
				log.ODM_ERROR('task_name  %s is not valid' % task_name)

			# setup values
			task = tasks[key]
			task.command = command
			task.num_inputs = num_inputs
			task.num_outputs = num_outputs
			task.inputs = inputs
			task.outputs = outputs

		return tasks


	def run_tasks(self):
		for id in range(self.initial_task_id, len(self.tasks)):
			# catch task with current id
			task = self.tasks[str(id)]
			# update task tracking
			self.current_task_id = task.id
			# run task
			task.state = task.run()


class ODMTask(object):
	"""docstring for ODMTask"""
	def __init__(self, id, name):
		# task definition
		self.id = id
		self.name = name
		self.command = None

		# task i/o
		self.num_inputs  = 0
		self.num_outputs = 0
		self.inputs = {}
		self.num_outputs = {}

		# Current task state (0:waiting, 1:running, 2:succeded: 3:failed)
		# By default we set a task in waiting state
		self.state = 0

	# Launch task
	def run(self):

		log.ODM_INFO('Running task %s %s ' % (self.id, self.name))

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

		method_call_str = str(self.command) + '()'
		try:
			eval(method_call_str)
		except Exception, e:
			log.ODM_ERROR('Method %s cannot be called' % method_call_str)
			log.ODM_ERROR(str(e))
		


		