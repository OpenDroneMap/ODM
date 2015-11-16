if __name__ == '__main__':
    
    objODMJob = prepare_objects()

    os.chdir(objODMJob.jobDir)

    dictSteps = {0:"resize", 1:"getKeypoints", 2:"match",
                 3:"bundler", 4:"cmvs", 5:"pmvs", 6:"odm_meshing",
                 7:"odm_texturing", 8:"odm_georeferencing",
                 9:"odm_orthophoto", 10:"zip_results"}
        
    listJobQueue = []
    intStart = -1
    intEnd = -2
    
    #  Create a dict for holding our steps
    #       key is step number, val is call
    #       Construct the calls below in eval by iterating through the composed dict of steps
    #       -->  Allows for running steps in any arbitray sequence, e.g. rebatching certain steps
    for keys in dictSteps.keys():
        if dictSteps[keys] == objODMJob.args.start_with:
            intStart = keys
        if dictSteps[keys] == objODMJob.args.end_with:
            intEnd = keys
    if intStart > intEnd:
        sys.stdout.writelines("No Valid Steps - Exitting.")
        exit(0)
    
    for i in range(intStart,intEnd,1):
        listJobQueue.append(dictSteps[i])
    
    for steps in listJobQueue:
        methodCall = steps+"()"
        strEval = "objODMJob."+methodCall
        #  EVAL safety - *only* internally generated strings (no user strings)
        eval(strEval)

    if args.zip_results:
        print "\nCompressing results - " + now()
        run("cd " + jobOptions["jobDir"] + "-results/ && tar -czf " +
            jobOptions["jobDir"] + "-results.tar.gz *")

 
    print "\n  - done - " + now()