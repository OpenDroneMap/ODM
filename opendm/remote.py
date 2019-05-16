import time
import datetime
import os
import sys
import threading
import signal
import zipfile
import glob
from opendm import log
from opendm import system
from pyodm import Node, exceptions
from pyodm.utils import AtomicCounter
from pyodm.types import TaskStatus
from osfm import OSFMContext, get_submodel_args_dict, get_submodel_argv
from pipes import quote

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
        self.params = {
            'tasks': [],
            'threads': []
        }
        self.node_online = True

        log.ODM_INFO("LRE: Initializing using cluster node %s:%s" % (self.node.host, self.node.port))
        try:
            odm_version = self.node.info().odm_version
            log.ODM_INFO("LRE: Node is online and running ODM version: %s"  % odm_version)
        except exceptions.NodeConnectionError:
            log.ODM_WARNING("LRE: The node seems to be offline! We'll still process the dataset, but it's going to run entirely locally.")
            self.node_online = False
        except Exception as e:
            log.ODM_ERROR("LRE: An unexpected problem happened while opening the node connection: %s" % str(e))
            exit(1)

    def set_projects(self, paths):
        self.project_paths = paths

    def run_reconstruction(self):
        self.run(ReconstructionTask)

    def run_toolchain(self):
        self.run(ToolchainTask)

    def run(self, taskClass):
        if not self.project_paths:
            return

        # Shared variables across threads
        class nonloc:
            error = None
            semaphore = None
            local_processing = False
        
        calculate_task_limit_lock = threading.Lock()
        finished_tasks = AtomicCounter(0)

        # Create queue
        q = queue.Queue()
        for pp in self.project_paths:
            log.ODM_DEBUG("LRE: Adding to queue %s" % pp)
            q.put(taskClass(pp, self.node, self.params))

        def remove_task_safe(task):
            try:
                removed = task.remove()
            except exceptions.OdmError:
                removed = False
            return removed
        
        def cleanup_remote_tasks():
            if self.params['tasks']:
                log.ODM_WARNING("LRE: Attempting to cleanup remote tasks")
            else:
                log.ODM_INFO("LRE: No remote tasks to cleanup")

            for task in self.params['tasks']:
                log.ODM_DEBUG("LRE: Removing remote task %s... %s" % (task.uuid, 'OK' if remove_task_safe(task) else 'NO'))

        def handle_result(task, local, error = None, partial=False):
            def cleanup_remote():
                if not partial and task.remote_task:
                    log.ODM_DEBUG("LRE: Cleaning up remote task (%s)... %s" % (task.remote_task.uuid, 'OK' if remove_task_safe(task.remote_task) else 'NO'))
                    self.params['tasks'].remove(task.remote_task)
                    task.remote_task = None

            if error:
                log.ODM_WARNING("LRE: %s failed with: %s" % (task, str(error)))
                
                # Special case in which the error is caused by a SIGTERM signal
                # this means a local processing was terminated either by CTRL+C or 
                # by canceling the task.
                if str(error) == "Child was terminated by signal 15":
                    system.exit_gracefully()

                created_semaphore = False
                if isinstance(error, NodeTaskLimitReachedException):
                    # Estimate the maximum number of tasks based on how many tasks
                    # are currently running
                    with calculate_task_limit_lock:
                        if not nonloc.semaphore:
                            node_task_limit = 0
                            for t in self.params['tasks']:
                                try:
                                    info = t.info()
                                    if info.status == TaskStatus.RUNNING and info.processing_time >= 0:
                                        node_task_limit += 1
                                except exceptions.OdmError:
                                    pass

                            sem_value = max(1, node_task_limit)
                            nonloc.semaphore = threading.Semaphore(sem_value)
                            log.ODM_DEBUG("LRE: Node task limit reached. Setting semaphore to %s" % sem_value)
                            for i in range(sem_value):
                                nonloc.semaphore.acquire()
                            created_semaphore = True

                # Retry, but only if the error is not related to a task failure
                if task.retries < task.max_retries and not isinstance(error, exceptions.TaskFailedError):
                    # Put task back in queue
                    # Don't increment the retry counter if this task simply reached the task
                    # limit count.
                    if not isinstance(error, NodeTaskLimitReachedException):
                        task.retries += 1
                    task.wait_until = datetime.datetime.now() + datetime.timedelta(seconds=task.retries * task.retry_timeout)
                    cleanup_remote()
                    q.task_done()

                    log.ODM_DEBUG("LRE: Re-queueing %s (retries: %s)" % (task, task.retries))
                    if not created_semaphore and nonloc.semaphore: nonloc.semaphore.release()
                    q.put(task)
                    return
                else:
                    nonloc.error = error
                    finished_tasks.increment()
            else:
                if not partial:
                    log.ODM_INFO("LRE: %s finished successfully" % task)
                    finished_tasks.increment()

            cleanup_remote()

            if not local and not partial and nonloc.semaphore: nonloc.semaphore.release()
            if not partial: q.task_done()
            
        def local_worker():
            while True:
                # Block until a new queue item is available
                task = q.get()

                if task is None or nonloc.error is not None:
                    q.task_done()
                    break

                # Process local
                try:
                    nonloc.local_processing = True
                    task.process(True, handle_result)
                except Exception as e:
                    handle_result(task, True, e)
                finally:
                    nonloc.local_processing = False


        def remote_worker():
            while True:
                had_semaphore = bool(nonloc.semaphore)

                # If we've found an estimate of the limit on the maximum number of tasks
                # a node can process, we block until some tasks have completed
                if nonloc.semaphore: nonloc.semaphore.acquire()

                # Block until a new queue item is available
                task = q.get()

                if task is None or nonloc.error is not None:
                    q.task_done()
                    if nonloc.semaphore: nonloc.semaphore.release()
                    break
                
                # Special case in which we've just created a semaphore
                if not had_semaphore and nonloc.semaphore:
                    log.ODM_INFO("LRE: Just found semaphore, sending %s back to the queue" % task)
                    q.put(task)
                    q.task_done()
                    continue

                # Yield to local processing
                if not nonloc.local_processing:
                    log.ODM_DEBUG("LRE: Yielding to local processing, sending %s back to the queue" % task)
                    q.put(task)
                    q.task_done()
                    if nonloc.semaphore: nonloc.semaphore.release()
                    time.sleep(0.05)
                    continue

                # Process remote
                try:
                    task.process(False, handle_result)
                except Exception as e:
                    handle_result(task, False, e)
        
        # Create queue thread
        local_thread = threading.Thread(target=local_worker)
        if self.node_online:
            remote_thread = threading.Thread(target=remote_worker)

        system.add_cleanup_callback(cleanup_remote_tasks)

        # Start workers
        local_thread.start()
        if self.node_online:
            remote_thread.start()

        # block until all tasks are done (or CTRL+C)
        try:
            while finished_tasks.value < len(self.project_paths) and nonloc.error is None:
                time.sleep(0.5)
        except KeyboardInterrupt:
            log.ODM_WARNING("LRE: CTRL+C")
            system.exit_gracefully()
        
        # stop workers
        if nonloc.semaphore: nonloc.semaphore.release()
        q.put(None)
        if self.node_online:
            q.put(None)

        # Wait for queue thread
        local_thread.join()
        if self.node_online:
            remote_thread.join()

        # Wait for all remains threads
        for thrds in self.params['threads']:
            thrds.join()
        
        system.remove_cleanup_callback(cleanup_remote_tasks)
        cleanup_remote_tasks()

        if nonloc.error is not None:
            # Try not to leak access token
            if isinstance(nonloc.error, exceptions.NodeConnectionError):
                raise exceptions.NodeConnectionError("A connection error happened. Check the connection to the processing node and try again.")
            else:
                raise nonloc.error
        

class NodeTaskLimitReachedException(Exception):
    pass

class Task:
    def __init__(self, project_path, node, params, max_retries=5, retry_timeout=10):
        self.project_path = project_path
        self.node = node
        self.params = params
        self.wait_until = datetime.datetime.now() # Don't run this task until a certain time
        self.max_retries = max_retries
        self.retries = 0
        self.retry_timeout = retry_timeout
        self.remote_task = None

    def process(self, local, done):
        def handle_result(error = None, partial=False):
            done(self, local, error, partial)

        log.ODM_INFO("LRE: About to process %s %s" % (self, 'locally' if local else 'remotely'))
        
        if local:
            self._process_local(handle_result) # Block until complete
        else:
            now = datetime.datetime.now()
            if self.wait_until > now:
                wait_for = (self.wait_until - now).seconds + 1
                log.ODM_DEBUG("LRE: Waiting %s seconds before processing %s" % (wait_for, self))
                time.sleep(wait_for)

            # TODO: we could consider uploading multiple tasks
            # in parallel. But since we are using the same node
            # perhaps this wouldn't be a big speedup.
            self._process_remote(handle_result) # Block until upload is complete

    def path(self, *paths):
        return os.path.join(self.project_path, *paths)

    def create_seed_payload(self, paths, touch_files=[]):
        paths = filter(os.path.exists, map(lambda p: self.path(p), paths))
        outfile = self.path("seed.zip")

        with zipfile.ZipFile(outfile, "w", compression=zipfile.ZIP_DEFLATED, allowZip64=True) as zf:
            for p in paths:
                if os.path.isdir(p):
                    for root, _, filenames in os.walk(p):
                        for filename in filenames:
                            filename = os.path.join(root, filename)
                            filename = os.path.normpath(filename)
                            zf.write(filename, os.path.relpath(filename, self.project_path))
                else:
                    zf.write(p, os.path.relpath(p, self.project_path))

            for tf in touch_files:
                zf.writestr(tf, "")

        return outfile

    def _process_local(self, done):
        try:
            self.process_local()
            done()
        except Exception as e:
            done(e)
    
    def _process_remote(self, done):
        try:
            self.process_remote(done)
            done(error=None, partial=True) # Upload is completed, but processing is not (partial)
        except Exception as e:
            done(e)

    def execute_remote_task(self, done, seed_files = [], seed_touch_files = [], outputs = []):
        """
        Run a task by creating a seed file with all files in seed_files, optionally
        creating empty files (for flag checks) specified in seed_touch_files
        and returning the results specified in outputs. Yeah it's pretty cool!
        """
        seed_file = self.create_seed_payload(seed_files, touch_files=seed_touch_files)
        
        # Find all images
        images = glob.glob(self.path("images/**"))

        # Add GCP (optional)
        if os.path.exists(self.path("gcp_list.txt")):
            images.append(self.path("gcp_list.txt"))
        
        # Add seed file
        images.append(seed_file)

        def print_progress(percentage):
            if percentage % 10 == 0:
                log.ODM_DEBUG("LRE: Upload of %s at [%s%%]" % (self, int(percentage)))
        
        # Upload task
        task = self.node.create_task(images, 
                get_submodel_args_dict(),
                progress_callback=print_progress,
                skip_post_processing=True,
                outputs=outputs)
        self.remote_task = task

        # Cleanup seed file
        os.remove(seed_file)

        # Keep track of tasks for cleanup
        self.params['tasks'].append(task)

        # Check status
        info = task.info()
        if info.status in [TaskStatus.RUNNING, TaskStatus.COMPLETED]:
            def monitor():
                class nonloc:
                    status_callback_calls = 0

                def status_callback(info):
                    # If a task switches from RUNNING to QUEUED, then we need to 
                    # stop the process and re-add the task to the queue.
                    if info.status == TaskStatus.QUEUED:
                        log.ODM_WARNING("LRE: %s (%s) turned from RUNNING to QUEUED. Re-adding to back of the queue." % (self, task.uuid))
                        raise NodeTaskLimitReachedException("Delayed task limit reached")
                    elif info.status == TaskStatus.RUNNING:
                        # Print a status message once in a while
                        nonloc.status_callback_calls += 1
                        if nonloc.status_callback_calls > 30:
                            log.ODM_DEBUG("LRE: %s (%s) is still running" % (self, task.uuid))
                            nonloc.status_callback_calls = 0
                try:
                    def print_progress(percentage):
                        if percentage % 10 == 0:
                            log.ODM_DEBUG("LRE: Download of %s at [%s%%]" % (self, int(percentage)))

                    task.wait_for_completion(status_callback=status_callback)
                    log.ODM_DEBUG("LRE: Downloading assets for %s" % self)
                    task.download_assets(self.project_path, progress_callback=print_progress)
                    log.ODM_DEBUG("LRE: Downloaded and extracted assets for %s" % self)
                    done()
                except exceptions.TaskFailedError as e:
                    # Try to get output
                    try:
                        msg = "(%s) failed with task output: %s" % (task.uuid, "\n".join(task.output()[-10:]))
                        done(exceptions.TaskFailedError(msg))
                    except:
                        log.ODM_WARNING("LRE: Could not retrieve task output for %s (%s)" % (self, task.uuid))
                        done(e)
                except Exception as e:
                    done(e)

            # Launch monitor thread and return
            t = threading.Thread(target=monitor)
            self.params['threads'].append(t)
            t.start()
        elif info.status == TaskStatus.QUEUED:
            raise NodeTaskLimitReachedException("Task limit reached")
        else:
            raise Exception("Could not send task to node, task status is %s" % str(info.status))

    
    def process_local(self):
        raise NotImplementedError()
    
    def process_remote(self, done):
        raise NotImplementedError()

    def __str__(self):
        return os.path.basename(self.project_path)


class ReconstructionTask(Task):
    def process_local(self):
        octx = OSFMContext(self.path("opensfm"))
        log.ODM_INFO("==================================")
        log.ODM_INFO("Local Reconstruction %s" % octx.name())
        log.ODM_INFO("==================================")
        octx.feature_matching()
        octx.reconstruct()
    
    def process_remote(self, done):
        self.execute_remote_task(done, seed_files=["opensfm/exif", 
                                            "opensfm/camera_models.json",
                                            "opensfm/reference_lla.json"],
                                 seed_touch_files=["opensfm/split_merge_stop_at_reconstruction.txt"],
                                 outputs=["opensfm/matches", "opensfm/features", 
                                          "opensfm/reconstruction.json",
                                          "opensfm/tracks.csv"])

class ToolchainTask(Task):
    def process_local(self):
        log.ODM_INFO("=============================")
        log.ODM_INFO("Local Toolchain %s" % self)
        log.ODM_INFO("=============================")

        submodel_name = os.path.basename(self.project_path)
        submodels_path = os.path.abspath(self.path(".."))
        project_name = os.path.basename(os.path.abspath(os.path.join(submodels_path, "..")))
        argv = get_submodel_argv(project_name, submodels_path, submodel_name)

        # Re-run the ODM toolchain on the submodel
        system.run(" ".join(map(quote, argv)), env_vars=os.environ.copy())

    
    def process_remote(self, done):
        self.execute_remote_task(done, seed_files=["opensfm/camera_models.json",
                                            "opensfm/reference_lla.json",
                                            "opensfm/reconstruction.json",
                                            "opensfm/tracks.csv"],
                            seed_touch_files=["opensfm/features/empty",
                                              "opensfm/matches/empty",
                                              "opensfm/exif/empty"],
                            outputs=["odm_orthophoto/odm_orthophoto.tif",
                                    "odm_orthophoto/cutline.gpkg",
                                    "odm_dem", 
                                    "odm_georeferencing"])