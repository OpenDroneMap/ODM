

def check_rerun(args, stage):
	return (args.rerun is not None and
                  args.rerun == stage) or \
                 (args.rerun_all) or \
                 (args.rerun_from is not None and
                  stage in args.rerun_from)