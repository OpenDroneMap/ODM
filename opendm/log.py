HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

def ODM_INFO(str):
	print OKBLUE + '[INFO] ' + str + ENDC

def ODM_WARNING(str):
	print WARNING + '[WARNING] ' + str + ENDC

def ODM_ERROR(str):
	print FAIL + '[ERROR] ' + str + ENDC
	
def ODM_DEBUG(str):
	print OKGREEN + '[DEBUG] ' + str + ENDC