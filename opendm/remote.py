from opendm import log
from pyodm import Node, exceptions

class HybridDistributedExecutor:
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
        
        



    def run_toolchain(self):
        if not self.project_paths:
            return
        

