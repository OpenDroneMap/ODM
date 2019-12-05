import time
import unittest
import threading
import random
from opendm.remote import LocalRemoteExecutor, Task, NodeTaskLimitReachedException
from pyodm import Node, exceptions
from pyodm.types import TaskStatus

class TestRemote(unittest.TestCase):
    def setUp(self):
        self.lre = LocalRemoteExecutor('http://localhost:9001')

        projects = []
        for i in range(9):
            projects.append('/submodels/submodel_00' + str(i).rjust(2, '0'))
        self.lre.set_projects(projects)

    def test_lre_init(self):
        self.assertFalse(self.lre.node_online)

    def test_processing_logic(self):
        # Fake online status
        self.lre.node_online = True

        MAX_QUEUE = 2
        class nonloc:
            local_task_check = False
            remote_queue = 1
            should_fail = False
            task_limit_reached = False

        class OdmTaskMock:
            def __init__(self, running, queue_num):
                self.running = running
                self.queue_num = queue_num
                self.uuid = 'xxxxx-xxxxx-xxxxx-xxxxx-xxxx' + str(queue_num)
            
            def info(self, with_output=None):
                class StatusMock:
                    status = TaskStatus.RUNNING if self.running else TaskStatus.QUEUED
                    processing_time = 1
                    output = "test output"
                return StatusMock()

            def remove(self):
                return True

        class TaskMock(Task):
            def process_local(self):
                # First task should be 0000 or 0001
                if not nonloc.local_task_check: nonloc.local_task_check = self.project_path.endswith("0000") or self.project_path.endswith("0001")
                
                if nonloc.should_fail:
                    if self.project_path.endswith("0006"):
                        raise exceptions.TaskFailedError("FAIL #6")
                        
                time.sleep(1)

            def process_remote(self, done):
                time.sleep(0.05) # file upload

                self.remote_task = OdmTaskMock(nonloc.remote_queue <= MAX_QUEUE, nonloc.remote_queue)
                self.params['tasks'].append(self.remote_task)
                
                if nonloc.should_fail:
                    if self.project_path.endswith("0006"):
                        raise exceptions.TaskFailedError("FAIL #6")
                    
                nonloc.remote_queue += 1

                # Upload successful
                done(error=None, partial=True)

                # Async processing
                def monitor():
                    try:
                        if nonloc.task_limit_reached and random.randint(0, 4) == 0:
                            nonloc.remote_queue -= 1
                            raise NodeTaskLimitReachedException("Random fail!")

                        if not nonloc.task_limit_reached and self.remote_task.queue_num > MAX_QUEUE:
                            nonloc.remote_queue -= 1
                            nonloc.task_limit_reached = True
                            raise NodeTaskLimitReachedException("Delayed task limit reached")
                        time.sleep(0.5)
                        nonloc.remote_queue -= 1
                        done()
                    except Exception as e:
                        done(e)

                t = threading.Thread(target=monitor)
                self.params['threads'].append(t)
                t.start()

        self.lre.run(TaskMock)
        self.assertTrue(nonloc.local_task_check)

        nonloc.should_fail = True
        nonloc.remote_queue = 1
        nonloc.task_limit_reached = False

        with self.assertRaises(exceptions.TaskFailedError):
            self.lre.run(TaskMock)

if __name__ == '__main__':
    unittest.main()