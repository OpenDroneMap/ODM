import time
import unittest
import threading
from opendm.remote import LocalRemoteExecutor, Task, NodeTaskLimitReachedException
from pyodm import Node
from pyodm.types import TaskStatus

class TestRemote(unittest.TestCase):
    def setUp(self):
        self.lre = LocalRemoteExecutor('http://invalid-host:3000')
        self.lre.set_projects(['/submodels/submodel_0000', 
                               '/submodels/submodel_0001',
                               '/submodels/submodel_0002',
                               '/submodels/submodel_0003',
                               '/submodels/submodel_0004',
                               '/submodels/submodel_0005',
                               '/submodels/submodel_0006',
                            ])

    def test_lre_init(self):
        self.assertFalse(self.lre.node_online)

    def test_processing_logic(self):
        # Fake online status
        self.lre.node_online = True

        MAX_QUEUE = 2
        class nonloc:
            local_task_check = False
            remote_queue = 1

        class OdmTaskMock:
            def __init__(self, running, queue_num):
                self.running = running
                self.queue_num = queue_num
                self.uuid = 'xxxxx-xxxxx-xxxxx-xxxxx-xxxx' + str(queue_num)
            
            def info(self):
                class StatusMock:
                    status = TaskStatus.RUNNING if self.running else TaskStatus.QUEUED
                return StatusMock()

            def remove(self):
                return True

        class TaskMock(Task):
            def process_local(self):
                # First task should be 0000 or 0001
                if not nonloc.local_task_check: nonloc.local_task_check = self.project_path.endswith("0000") or self.project_path.endswith("0001")
                time.sleep(1)

            def process_remote(self, done):
                time.sleep(0.05) # file upload

                self.remote_task = OdmTaskMock(nonloc.remote_queue <= MAX_QUEUE, nonloc.remote_queue)
                self.params['tasks'].append(self.remote_task)
                nonloc.remote_queue += 1

                # Upload successful
                done(error=None, partial=True)

                # Async processing
                def monitor():
                    time.sleep(0.2)

                    try:
                        if self.remote_task.queue_num > MAX_QUEUE:
                            nonloc.remote_queue -= 1
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

