import time
import datetime
import os
import sys
import threading
import signal
from opendm import log
from pyodm import Node, exceptions
from pyodm.utils import AtomicCounter

try:
    import queue
except ImportError:
    import Queue as queue

class LocalRemoteExecutor:
    """
    A class for performing OpenSfM reconstructions and full ODM pipeline executions
    using a mix of local and remote processing. Tasks are executed locally one at a time
    and remotely until a node runs out of available slots for processing. This allows us
    to use the processing power of the current machine as well as offloading tasks to a 
    network node.
    """
    def __init__(self, nodeUrl):
        self.node = Node.from_url(nodeUrl)

        log.ODM_INFO("LRE: Initializing using cluster node %s:%s" % (self.node.host, self.node.port))
        try:
            odm_version = self.node.info().odm_version
            log.ODM_INFO("LRE: Node is online and running ODM version: %s"  % odm_version)
        except exceptions.NodeConnectionError:
            log.ODM_WARNING("LRE: The node seems to be offline! We'll still process the dataset, but it's going to run entirely locally.")
        except Exception as e:
            log.ODM_ERROR("LRE: An unexpected problem happened while opening the node connection: %s" % str(e))
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
            semaphore = None
        
        node_task_limit = AtomicCounter(0)

        # Create queue
        q = queue.Queue()
        for pp in self.project_paths:
            log.ODM_DEBUG("LRE: Adding to queue %s" % pp)
            q.put(ReconstructionTask(pp))
        
        def handle_result(task, local, error = None):
            if error:
                if isinstance(error, NodeTaskLimitReachedException) and not nonloc.semaphore:
                    nonloc.semaphore = threading.Semaphore(node_task_limit.value)
                    log.ODM_DEBUG("LRE: Node task limit reached. Setting semaphore to %s" % node_task_limit.value)
                    for i in range(node_task_limit.value):
                        nonloc.semaphore.acquire()
                
                log.ODM_WARNING("LRE: %s failed with: %s" % (task, str(error)))

                if task.retries < task.max_retries:
                    # Put task back in queue
                    task.retries += 1
                    task.wait_until = datetime.datetime.now() + datetime.timedelta(seconds=task.retries * task.retry_timeout)
                    log.ODM_DEBUG("LRE: Re-queueing %s (retries: %s)" % (task, task.retries))
                    q.put(task)
                else:
                    nonloc.error = e
            else:
                if not local:
                    node_task_limit.increment(-1)

                log.ODM_INFO("LRE: %s finished successfully" % task)

            if local:
                nonloc.local_is_processing = False              

            if nonloc.semaphore: nonloc.semaphore.release()
            q.task_done()

        def worker():
            while True:
                # If we've found a limit on the maximum number of tasks
                # a node can process, we block until some tasks have completed
                if nonloc.semaphore: nonloc.semaphore.acquire()

                task = q.get()
                if task is None or nonloc.error is not None:
                    q.task_done()
                    if nonloc.semaphore: nonloc.semaphore.release()
                    break

                if not nonloc.local_is_processing:
                    # Process local
                    try:
                        nonloc.local_is_processing = True
                        task.process(True, handle_result)
                    except Exception as e:
                        handle_result(task, True, e)
                else:
                    # Process remote
                    try:
                        task.process(False, handle_result)
                        node_task_limit.increment() # Called after upload, but before processing is started
                    except Exception as e:
                        handle_result(task, False, e)
        
        # Define thread
        t = threading.Thread(target=worker)

        def cleanup_remote_tasks():
            log.ODM_WARNING("LRE: Attempting to cleanup remote tasks")
            pass # TODO

        # Capture SIGTERM so that we can 
        # attempt to cleanup if the process is terminated
        original_sigterm_handler = signal.getsignal(signal.SIGTERM)

        def sigterm_handler(signum, frame):
            log.ODM_WARNING("LRE: Caught SIGTERM")
            cleanup_remote_tasks()
            os._exit(1)

        signal.signal(signal.SIGTERM, sigterm_handler)

        # Start worker process
        t.start()

        # block until all tasks are done (or CTRL+C)
        try:
            while q.unfinished_tasks > 0:
                time.sleep(0.5)
        except KeyboardInterrupt:
            log.ODM_WARNING("LRE: CTRL+C")
            cleanup_remote_tasks()
            os._exit(1)
        
        # stop workers
        q.put(None)

        # Wait for thread
        t.join()

        # restore SIGTERM handler
        signal.signal(signal.SIGTERM, original_sigterm_handler)

        if nonloc.error is not None:
            raise nonloc.error


    def run_toolchain(self):
        if not self.project_paths:
            return
        

class NodeTaskLimitReachedException(Exception):
    pass

class Task:
    def __init__(self, project_path, max_retries=10, retry_timeout=1):
        self.project_path = project_path
        self.wait_until = datetime.datetime.now() # Don't run this task until a certain time
        self.max_retries = max_retries
        self.retries = 0
        self.retry_timeout = retry_timeout
        self.local = None

    def process(self, local, done):
        def handle_result(error = None):
            done(self, local, error)

        log.ODM_INFO("LRE: About to process %s %s" % (self, 'locally' if local else 'remotely'))
        
        if local:
            t = threading.Thread(target=self.process_local, args=(handle_result, ))
            t.start()
        else:
            now = datetime.datetime.now()
            if self.wait_until > now:
                wait_for = (self.wait_until - now).seconds
                log.ODM_DEBUG("LRE: Waiting %s seconds before processing %s" % (wait_for, self))
                time.sleep(wait_for)

            # TODO: we could consider uploading multiple tasks
            # in parallel. But since we are using the same node
            # perhaps this wouldn't be a big speedup.
            self.process_remote(handle_result) # Block until upload is complete

    def process_local(self, done):
        raise NotImplementedError()
    
    def process_remote(self, done):
        raise NotImplementedError()

    def __str__(self):
        return os.path.basename(self.project_path)

class ReconstructionTask(Task):
    def process_local(self, done):
        print("Process local: " + self.project_path)
        time.sleep(10)
        done()
    
    def process_remote(self, done):

        def test():
            time.sleep(4)
            done()

        if self.project_path == '/datasets/brighton/opensfm/submodels/submodel_0001':
            done(Exception("TEST EXCEPTION!" + self.project_path))
        elif self.project_path == '/datasets/brighton/opensfm/submodels/submodel_0002':
            done(NodeTaskLimitReachedException("Limit reached"))
        elif self.project_path == '/datasets/brighton/opensfm/submodels/submodel_0003':
            threading.Thread(target=test).start()
        elif self.project_path == '/datasets/brighton/opensfm/submodels/submodel_0004':
            threading.Thread(target=test).start()
        else:
            print("Process remote: " + self.project_path) 
            done()

