import system

def cmvs():
    """Run CMVS"""
    print "\n  - running cmvs - " + system.now()

    os.chdir(jobOptions["jobDir"])

    run("\"" + BIN_PATH + "/cmvs\" pmvs/ " + str(args['--cmvs-maxImages'])        \
        + " " + str(CORES))
    run("\"" + BIN_PATH + "/genOption\" pmvs/ " + str(args['--pmvs-level'])       \
        + " " + str(args['--pmvs-csize']) + " " + str(args['--pmvs-threshold'])   \
        + " " + str(args['--pmvs-wsize']) + " " + str(args['--pmvs-minImageNum']) \
        + " " + str(CORES))

    if args['--end-with'] != "cmvs":
        pmvs()


def pmvs():
    """Run PMVS"""
    print "\n  - running pmvs - " + system.now()

    os.chdir(jobOptions["jobDir"])

    run("\"" + BIN_PATH + "/pmvs2\" pmvs/ option-0000")

    run("cp -Rf \"" + jobOptions["jobDir"] + "/pmvs/models\" \"" + jobOptions["jobDir"] + "-results\"")

 
    if args['--end-with'] != "pmvs":
        odm_meshing()