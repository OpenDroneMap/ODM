import time
import unittest
import threading
from opendm.remote import LocalRemoteExecutor, Task, NodeTaskLimitReachedException
from pyodm import Node

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
            remote_queue = 0

        class TaskMock(Task):
            def process_local(self):
                # First task should be submodel_0000
                if not nonloc.local_task_check: nonloc.local_task_check = self.project_path.endswith("0000")
                time.sleep(3)

            def process_remote(self, done):
                time.sleep(0.2)

                # Upload successful
                done(error=None, partial=True)

                # Async processing
                def monitor():
                    nonloc.remote_queue += 1
                    time.sleep(0.3)

                    try:
                        if nonloc.remote_queue > MAX_QUEUE:
                            nonloc.remote_queue = 0
                            raise NodeTaskLimitReachedException("Delayed task limit reached")
                        done()
                    except Exception as e:
                        done(e)

                t = threading.Thread(target=monitor)
                self.params['threads'].append(t)
                t.start()

        self.lre.run(TaskMock)
        self.assertTrue(nonloc.local_task_check)

