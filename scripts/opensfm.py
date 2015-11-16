def opensfm():
    print "\n  - running OpenSfM - " + now()

    os.chdir(jobOptions["jobDir"])

    # Create bundler's list.txt
    filesList = ""
    for fileObject in objects:
        if fileObject["isOk"]:
            filesList += "./" + fileObject["src"] + " 0 {:.5f}\n".format(fileObject["focalpx"])
    filesList = filesList.rstrip('\n')

    with open(jobOptions["step_3_filelist"], 'w') as fout:
        fout.write(filesList)

    # Create opensfm working folder
    mkdir_p("opensfm")

    # Configure OpenSfM
    config = [
       "use_exif_size: no",
       "feature_process_size: {}".format(jobOptions["resizeTo"]),
       "feature_min_frames: {}".format(args.min_num_features),
       "processes: {}".format(CORES),
    ]
    if args.matcher_preselect:
        config.append("matching_gps_neighbors: {}".format(args.matcher_k))

    with open('opensfm/config.yaml', 'w') as fout:
        fout.write("\n".join(config))

    print 'running import_bundler'
    # Convert bundler's input to opensfm
    run('PYTHONPATH={} "{}/bin/import_bundler" opensfm --list list.txt'.format(PYOPENCV_PATH, OPENSFM_PATH))

    # Run OpenSfM reconstruction
    run('PYTHONPATH={} "{}/bin/run_all" opensfm'.format(PYOPENCV_PATH, OPENSFM_PATH))

    # Convert back to bundler's format
    run('PYTHONPATH={} "{}/bin/export_bundler" opensfm'.format(PYOPENCV_PATH, OPENSFM_PATH))

    bundler_to_pmvs("opensfm/bundle_r000.out")

    if args.end_with != "bundler":
        cmvs()