from opendm import log

class HybridDistributedExecutor:
    def __init__(self, nodeUrl):
        self.nodeUrl = nodeUrl
        log.ODM_INFO("Initializing hybrid distributed executor with cluster node: %s" % nodeUrl)

    def set_projects(self, paths):
        self.project_paths = paths

    def run_reconstruct(self):
        print(self.project_paths)
        exit(1)

    def run_toolchain(self):
        pass
    
