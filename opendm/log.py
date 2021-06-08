import sys
import threading
import os
import json
import datetime

from opendm.loghelpers import double_quote, args_to_dict

if sys.platform == 'win32':
    # No colors on Windows, sorry!
    HEADER = ''
    OKBLUE = ''
    OKGREEN = ''
    DEFAULT = ''
    WARNING = ''
    FAIL = ''
    ENDC = ''
else:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    DEFAULT = '\033[39m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

lock = threading.Lock()

def odm_version():
    with open(os.path.join(os.path.dirname(__file__), "..", "VERSION")) as f:
        return f.read().split("\n")[0].strip()

class ODMLogger:
    def __init__(self):
        self.show_debug = False
        self.json = None
        self.json_output_file = None
        self.start_time = datetime.datetime.now()

    def log(self, startc, msg, level_name):
        level = ("[" + level_name + "]").ljust(9)
        with lock:
            print("%s%s %s%s" % (startc, level, msg, ENDC))
            sys.stdout.flush()
    
    def init_json_output(self, output_file, args):
        self.json_output_file = output_file
        self.json = {}
        self.json['odmVersion'] = odm_version()
        self.json['options'] = args_to_dict(args)
        self.json["startTime"] = self.start_time.isoformat()
        self.json["stages"] = []
            
    def info(self, msg):
        self.log(DEFAULT, msg, "INFO")

    def warning(self, msg):
        self.log(WARNING, msg, "WARNING")

    def error(self, msg):
        self.log(FAIL, msg, "ERROR")

    def exception(self, msg):
        self.log(FAIL, msg, "EXCEPTION")

    def debug(self, msg):
        if self.show_debug:
            self.log(OKGREEN, msg, "DEBUG")

    def close(self):
        if self.json is not None and self.json_output_file is not None:
            with open(self.json_output_file, 'w') as f:
                f.write(json.dumps(self.json, indent=4))

logger = ODMLogger()

ODM_INFO = logger.info
ODM_WARNING = logger.warning
ODM_ERROR = logger.error
ODM_EXCEPTION = logger.exception
ODM_DEBUG = logger.debug
