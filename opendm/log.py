import sys
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

# logging has too many quirks...
class ODMLogger:
    def log(self, startc, msg, level_name):
        level = ("[" + level_name + "]").ljust(9)
        print("%s%s %s%s" % (startc, level, msg, ENDC))
        sys.stdout.flush()

    def info(self, msg):
        self.log(OKBLUE, msg, "INFO")

    def warning(self, msg):
        self.log(WARNING, msg, "WARNING")

    def error(self, msg):
        self.log(FAIL, msg, "ERROR")

    def exception(self, msg):
        self.log(FAIL, msg, "EXCEPTION")

    def debug(self, msg):
        self.log(OKGREEN, msg, "DEBUG")

logger = ODMLogger()

ODM_INFO = logger.info
ODM_WARNING = logger.warning
ODM_ERROR = logger.error
ODM_EXCEPTION = logger.exception
ODM_DEBUG = logger.debug
