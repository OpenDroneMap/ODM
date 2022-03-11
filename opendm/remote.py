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
from opendm import config
from pyodm import Node, exceptions
from pyodm.utils import AtomicCounter
from pyodm.types import TaskStatus
from opendm.osfm import OSFMContext, get_submodel_args_dict, get_submodel_argv
from opendm.utils import double_quote

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
    def __init__(self, nodeUrl, rerun = False):
        self.node = Node.from_url(nodeUrl)
        self.params = {
            'tasks': [],
            'threads': [],
            'rerun': rerun
        }
        self.node_online = True

        log.ODM_INFO("LRE: Initializing using cluster node %s:%s" % (self.node.host, self.node.port))
        try:
            info = self.node.info()
            log.ODM_INFO("LRE: Node is online and running %s version %s"  % (info.engine, info.engine_version))
        except exceptions.NodeConnectionError:
            log.ODM_WARNING("LRE: The node seems to be offline! We'll still process the dataset, but it's going to run entirely locally.")
            self.node_online = False
        except Exception as e:
            raise system.ExitException("LRE: An unexpected problem happened while opening the node connection: %s" % str(e))

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
            local_processing = False
            max_remote_tasks = None
        
        calculate_task_limit_lock = threading.Lock()
        finished_tasks = AtomicCounter(0)
        remote_running_tasks = AtomicCounter(0)

        # Create queue
        q = queue.Queue()
        for pp in self.project_paths:
            log.ODM_INFO("LRE: Adding to queue %s" % pp)
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
                log.ODM_INFO("LRE: No remote tasks left to cleanup")

            for task in self.params['tasks']:
                log.ODM_INFO("LRE: Removing remote task %s... %s" % (task.uuid, 'OK' if remove_task_safe(task) else 'NO'))

        def handle_result(task, local, error = None, partial=False):
            def cleanup_remote():
                if not partial and task.remote_task:
                    log.ODM_INFO("LRE: Cleaning up remote task (%s)... %s" % (task.remote_task.uuid, 'OK' if remove_task_safe(task.remote_task) else 'NO'))
                    self.params['tasks'].remove(task.remote_task)
                    task.remote_task = None

            if error:
                log.ODM_WARNING("LRE: %s failed with: %s" % (task, str(error)))
                
                # Special case in which the error is caused by a SIGTERM signal
                # this means a local processing was terminated either by CTRL+C or 
                # by canceling the task.
                if str(error) == "Child was terminated by signal 15":
                    system.exit_gracefully()

                task_limit_reached = isinstance(error, NodeTaskLimitReachedException)
                if task_limit_reached:
                    # Estimate the maximum number of tasks based on how many tasks
                    # are currently running
                    with calculate_task_limit_lock:
                        if nonloc.max_remote_tasks is None:
                            node_task_limit = 0
                            for t in self.params['tasks']:
                                try:
                                    info = t.info(with_output=-3)
                                    if info.status == TaskStatus.RUNNING and info.processing_time >= 0 and len(info.output) >= 3:
                                        node_task_limit += 1
                                except exceptions.OdmError:
                                    pass

                            nonloc.max_remote_tasks = max(1, node_task_limit)
                            log.ODM_INFO("LRE: Node task limit reached. Setting max remote tasks to %s" % node_task_limit)
                                

                # Retry, but only if the error is not related to a task failure
                if task.retries < task.max_retries and not isinstance(error, exceptions.TaskFailedError):
                    # Put task back in queue
                    # Don't increment the retry counter if this task simply reached the task
                    # limit count.
                    if not task_limit_reached:
                        task.retries += 1
                    task.wait_until = datetime.datetime.now() + datetime.timedelta(seconds=task.retries * task.retry_timeout)
                    cleanup_remote()
                    q.task_done()

                    log.ODM_INFO("LRE: Re-queueing %s (retries: %s)" % (task, task.retries))
                    q.put(task)
                    if not local: remote_running_tasks.increment(-1)
                    return
                else:
                    nonloc.error = error
                    finished_tasks.increment()
                    if not local: remote_running_tasks.increment(-1)
            else:
                if not partial:
                    log.ODM_INFO("LRE: %s finished successfully" % task)
                    finished_tasks.increment()
                    if not local: remote_running_tasks.increment(-1)

            cleanup_remote()
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
                # Block until a new queue item is available
                task = q.get()

                if task is None or nonloc.error is not None:
                    q.task_done()
                    break
                
                # Yield to local processing
                if not nonloc.local_processing:
                    log.ODM_INFO("LRE: Yielding to local processing, sending %s back to the queue" % task)
                    q.put(task)
                    q.task_done()
                    time.sleep(0.05)
                    continue

                # If we've found an estimate of the limit on the maximum number of tasks
                # a node can process, we block until some tasks have completed
                if nonloc.max_remote_tasks is not None and remote_running_tasks.value >= nonloc.max_remote_tasks:
                    q.put(task)
                    q.task_done()
                    time.sleep(2)
                    continue

                # Process remote
                try:
                    remote_running_tasks.increment()
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
                log.ODM_INFO("LRE: Waiting %s seconds before processing %s" % (wait_for, self))
                time.sleep(wait_for)

            # TODO: we could consider uploading multiple tasks
            # in parallel. But since we are using the same node
            # perhaps this wouldn't be a big speedup.
            self._process_remote(handle_result) # Block until upload is complete

    def path(self, *paths):
        return os.path.join(self.project_path, *paths)

    def touch(self, file):
        with open(file, 'w') as fout:
            fout.write("Done!\n")

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

    def execute_remote_task(self, done, seed_files = [], seed_touch_files = [], outputs = [], ):
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
        
        # Add GEO (optional)
        if os.path.exists(self.path("geo.txt")):
            images.append(self.path("geo.txt"))
        
        # Add seed file
        images.append(seed_file)

        class nonloc:
            last_update = 0

        def print_progress(percentage):
            if (time.time() - nonloc.last_update >= 2) or int(percentage) == 100:
                log.ODM_INFO("LRE: Upload of %s at [%s%%]" % (self, int(percentage)))
                nonloc.last_update = time.time()

        # Upload task
        task = self.node.create_task(images, 
                get_submodel_args_dict(config.config()),
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
                    last_update = 0

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
                            log.ODM_INFO("LRE: %s (%s) is still running" % (self, task.uuid))
                            nonloc.status_callback_calls = 0
                try:
                    def print_progress(percentage):
                        if (time.time() - nonloc.last_update >= 2) or int(percentage) == 100:
                            log.ODM_INFO("LRE: Download of %s at [%s%%]" % (self, int(percentage)))
                            nonloc.last_update = time.time()

                    task.wait_for_completion(status_callback=status_callback)
                    log.ODM_INFO("LRE: Downloading assets for %s" % self)
                    task.download_assets(self.project_path, progress_callback=print_progress)
                    log.ODM_INFO("LRE: Downloaded and extracted assets for %s" % self)
                    done()
                except exceptions.TaskFailedError as e:
                    # Try to get output
                    try:
                        output_lines = task.output()

                        # Save to file
                        error_log_path = self.path("error.log")
                        with open(error_log_path, 'w') as f:
                            f.write('\n'.join(output_lines) + '\n')

                        msg = "(%s) failed with task output: %s\nFull log saved at %s" % (task.uuid, "\n".join(output_lines[-10:]), error_log_path)
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
        octx.feature_matching(self.params['rerun'])
        octx.reconstruct(self.params['rerun'])
    
    def process_remote(self, done):
        octx = OSFMContext(self.path("opensfm"))
        if not octx.is_feature_matching_done() or not octx.is_reconstruction_done() or self.params['rerun']:
            self.execute_remote_task(done, seed_files=["opensfm/exif", 
                                                "opensfm/camera_models.json",
                                                "opensfm/reference_lla.json"],
                                    seed_touch_files=["opensfm/split_merge_stop_at_reconstruction.txt"],
                                    outputs=["opensfm/matches", "opensfm/features", 
                                            "opensfm/reconstruction.json",
                                            "opensfm/tracks.csv",
                                            "cameras.json"])
        else:
            log.ODM_INFO("Already processed feature matching and reconstruction for %s" % octx.name())
            done()

class ToolchainTask(Task):
    def process_local(self):
        completed_file = self.path("toolchain_completed.txt")
        submodel_name = os.path.basename(self.project_path)
        
        if not os.path.exists(completed_file) or self.params['rerun']:
            log.ODM_INFO("=============================")
            log.ODM_INFO("Local Toolchain %s" % self)
            log.ODM_INFO("=============================")

            submodels_path = os.path.abspath(self.path(".."))
            argv = get_submodel_argv(config.config(), submodels_path, submodel_name)

            # Re-run the ODM toolchain on the submodel
            system.run(" ".join(map(double_quote, map(str, argv))), env_vars=os.environ.copy())

            # This will only get executed if the command above succeeds
            self.touch(completed_file)
        else:
            log.ODM_INFO("Already processed toolchain for %s" % submodel_name)
    
    def process_remote(self, done):
        completed_file = self.path("toolchain_completed.txt")
        submodel_name = os.path.basename(self.project_path)

        def handle_result(error = None):
            # Mark task as completed if no error
            if error is None:
                self.touch(completed_file)
            done(error=error)

        if not os.path.exists(completed_file) or self.params['rerun']:
            self.execute_remote_task(handle_result, seed_files=["opensfm/camera_models.json",
                                                "opensfm/reference_lla.json",
                                                "opensfm/reconstruction.json",
                                                "opensfm/tracks.csv"],
                                seed_touch_files=["opensfm/features/empty",
                                                "opensfm/matches/empty",
                                                "opensfm/exif/empty"],
                                outputs=["odm_orthophoto/odm_orthophoto.tif",
                                        "odm_orthophoto/cutline.gpkg",
                                        "odm_orthophoto/odm_orthophoto_cut.tif",
                                        "odm_orthophoto/odm_orthophoto_feathered.tif",
                                        "odm_dem",
                                        "odm_report",
                                        "odm_georeferencing"])
        else:
            log.ODM_INFO("Already processed toolchain for %s" % submodel_name)
            handle_result()