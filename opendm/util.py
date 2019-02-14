

def is_run(args, stage):
	"""
	returns true if the module is to be forced to run regardless of state. This is true if:
		1. The metadata object has this module's run status as false
		2. The --rerun argument equals "stage"
		3. "stage" is in the -rerun-from arg list
		4. --rerun-all is True
	"""
	return (args.rerun is not None and
                  args.rerun == stage) or \
                 (args.rerun_all) or \
                 (args.rerun_from is not None and
                  stage in args.rerun_from)