import time
import datetime
import threading
from opendm import log
from pyodm import Node, exceptions
try:
    import queue
except ImportError:
    import Queue as queue

class HybridDistributedExecutor:
    """
    A class for performing OpenSfM reconstructions and full ODM pipeline executions
    using a mix of local and remote processing. Tasks are executed locally one at a time
    and remotely until a node runs out of available slots for processing. This allows us
    to use the processing power of the current machine as well as offloading tasks to a 
    network node.
    """
    def __init__(self, nodeUrl):
        self.node = Node.from_url(nodeUrl)

        log.ODM_INFO("Initializing hybrid distributed executor using cluster node %s" % nodeUrl)
        try:
            odm_version = self.node.info().odm_version
            log.ODM_INFO("Node is online and running ODM version: %s"  % odm_version)
        except exceptions.NodeConnectionError:
            log.ODM_WARNING("The node seems to be offline! We'll still process the dataset, but it's going to run entirely locally.")
        except Exception as e:
            log.ODM_ERROR("An unexpected problem happened while opening the node connection: %s" % str(e))
            exit(1)

    def set_projects(self, paths):
        self.project_paths = paths

    def run_reconstruction(self):
        if not self.project_paths:
            return

        # Shared variables across threads
        class nonloc:
            error = None
            local_is_processing = False

        # Create queue
        q = queue.Queue()
        for pp in self.project_paths:
            q.put(ReconstructionTask(pp))
        
        def handle_result(task, local, error = None):
            if error:
                print("ERROR!!! " + str(error))
                if task.retries < task.max_retries:
                    # Put task back in queue
                    task.retries += 1
                    task.wait_until = datetime.datetime.now() + datetime.timedelta(seconds=task.retries * task.retry_timeout)
                    q.put(task)
                else:
                    nonloc.error = e
            
            if local:
                nonloc.local_is_processing = False

            q.task_done()

        def worker():
            while True:
                task = q.get()
                if task is None or nonloc.error is not None:
                    q.task_done()
                    break
                
                if not nonloc.local_is_processing:
                    # Process local
                    nonloc.local_is_processing = True
                    task.process(True, handle_result)
                else:
                    # Process remote
                    now = datetime.datetime.now()
                    if task.wait_until > now:
                        time.sleep((task.wait_until - now).seconds)

                    task.process(False, handle_result)

        t = threading.Thread(target=worker)
        t.start()

        # block until all tasks are done
        q.join()

        # stop workers
        q.put(None)

        t.join()

        if nonloc.error is not None:
            raise nonloc.error


    def run_toolchain(self):
        if not self.project_paths:
            return
        

class Task:
    def __init__(self, project_path, max_retries=10, retry_timeout=10):
        self.project_path = project_path
        self.wait_until = datetime.datetime.now() # Don't run this task until a certain time
        self.max_retries = max_retries
        self.retries = 0
        self.retry_timeout = retry_timeout
        self.local = None

    def process(self, local, done):
        def handle_result(error = None):
            done(self, local, error)

        t = threading.Thread(target=getattr(self, 'process_local' if local else 'process_remote'), args=(handle_result, ))
        t.start()

    def process_local(self, done):
        raise NotImplementedError()
    
    def process_remote(self, done):
        raise NotImplementedError()

class ReconstructionTask(Task):

    def process_local(self, done):
        print("Process local: " + self.project_path)
        time.sleep(0.1)
        done()
    
    def process_remote(self, done):
        time.sleep(0.3)

        if self.project_path == '/datasets/brighton/opensfm/submodels/submodel_0001':
            done(Exception("TEST EXCEPTION!" + self.project_path))
        else:
            print("Process remote: " + self.project_path) 
            done()