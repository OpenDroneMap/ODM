import system

def odm_meshing():
    """Run odm_meshing"""
    print "\n  - running meshing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_meshing")
    except:
        pass

 
    run("\"" + BIN_PATH + "/odm_meshing\" -inputFile " + jobOptions["jobDir"]     \
        + "-results/option-0000.ply -outputFile " + jobOptions["jobDir"]          \
        + "-results/odm_mesh-0000.ply -logFile " + jobOptions["jobDir"]           \
        + "/odm_meshing/odm_meshing_log.txt -maxVertexCount "                     \
        + str(args['--odm_meshing-maxVertexCount']) + " -octreeDepth "            \
        + str(args['--odm_meshing-octreeDepth']) + " -samplesPerNode "            \
        + str(args['--odm_meshing-samplesPerNode']) + " -solverDivide "           \
        + str(args['--odm_meshing-solverDivide']))

    if args['--end-with'] != "odm_meshing":
        odm_texturing()