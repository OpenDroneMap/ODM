def odm_texturing():
    """Run odm_texturing"""
    print "\n  - running texturing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_texturing")
        os.mkdir(jobOptions["jobDir"] + "-results/odm_texturing")
    except:
        pass

 
    run("\"" + BIN_PATH + "/odm_texturing\" -bundleFile " + jobOptions["jobDir"] + "/pmvs/bundle.rd.out -imagesPath "\
        + jobOptions["srcDir"] + "/ -imagesListPath " + jobOptions["jobDir"] + "/pmvs/list.rd.txt -inputModelPath "  \
        + jobOptions["jobDir"] + "-results/odm_mesh-0000.ply -outputFolder " + jobOptions["jobDir"]                  \
        + "-results/odm_texturing/ -textureResolution " + str(args['--odm_texturing-textureResolution'])             \
        + " -bundleResizedTo " + str(jobOptions["resizeTo"]) + " -textureWithSize " +                                \
        str(args['--odm_texturing-textureWithSize']) + " -logFile " + jobOptions["jobDir"]                           \
        + "/odm_texturing/odm_texturing_log.txt")

    if args['--end-with'] != "odm_texturing":
        odm_georeferencing()