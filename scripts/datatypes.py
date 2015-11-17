from tasks import ODMTaskManager

class ODMApp:
    '''   ODMJob - a class for ODM Activities
    '''
    def __init__(self, images_dir, args=None):
        # Internal app config
        self.args = args
        self.images_dir = images_dir
        
        # Task manager
        # configure and schedule tasks
        self.task_manager = ODMTaskManager()

    # Run all tasks given an starting point
    def run(self, initial_task_id):

        self.task_manager.initial_task_id = initial_task_id
        self.task_manager.run_tasks()