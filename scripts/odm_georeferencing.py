def odm_georeferencing():
    """Run odm_georeferencing"""
    print "\n  - running georeferencing - " + now()

    os.chdir(jobOptions["jobDir"])
    try:
        os.mkdir(jobOptions["jobDir"] + "/odm_georeferencing")
    except:
        pass

    if not args.odm_georeferencing_useGcp:
        run("\"" + BIN_PATH + "/odm_extract_utm\" -imagesPath " + jobOptions["srcDir"] + "/ -imageListFile " \
            + jobOptions["jobDir"] + "/pmvs/list.rd.txt -outputCoordFile " + jobOptions["jobDir"]            \
            + "/odm_georeferencing/coordFile.txt")
        
        run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + jobOptions["jobDir"]                            \
            + "/pmvs/bundle.rd.out -inputCoordFile " + jobOptions["jobDir"]                                  \
            + "/odm_georeferencing/coordFile.txt -inputFile " + jobOptions["jobDir"]                         \
            + "-results/odm_texturing/odm_textured_model.obj -outputFile " + jobOptions["jobDir"]            \
            + "-results/odm_texturing/odm_textured_model_geo.obj -inputPointCloudFile "                      \
            + jobOptions["jobDir"] + "-results/option-0000.ply -outputPointCloudFile " + jobOptions["jobDir"] \
            + "-results/option-0000_georef.ply -logFile " + jobOptions["jobDir"]                             \
            + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + jobOptions["jobDir"] \
            + "-results/odm_texturing/odm_textured_model_geo_georef_system.txt")
        
    elif os.path.isfile(jobOptions["srcDir"] + "/" + args.odm_georeferencing_gcpFile):
        run("\"" + BIN_PATH + "/odm_georef\" -bundleFile " + jobOptions["jobDir"]                             \
            + "/pmvs/bundle.rd.out -gcpFile " + jobOptions["srcDir"] + "/" + args.odm_georeferencing_gcpFile  \
            + " -imagesPath " + jobOptions["srcDir"] + "/ -imagesListPath " + jobOptions["jobDir"]            \
            + "/pmvs/list.rd.txt -bundleResizedTo " + str(jobOptions["resizeTo"]) + " -inputFile "            \
            + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model.obj -outputFile "             \
            + jobOptions["jobDir"] + "-results/odm_texturing/odm_textured_model_geo.obj -outputCoordFile "    \
            + jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt -inputPointCloudFile "                \
            + jobOptions["jobDir"] + "-results/option-0000.ply -outputPointCloudFile " + jobOptions["jobDir"] \
            + "-results/option-0000_georef.ply -logFile " + jobOptions["jobDir"]                              \
            + "/odm_georeferencing/odm_georeferencing_log.txt -georefFileOutputPath " + jobOptions["jobDir"]  \
            + "-results/odm_texturing/odm_textured_model_geo_georef_system.txt")
    else:
        print "Warning: No GCP file. Consider rerunning with argument --odm_georeferencing-useGcp false --start-with odm_georeferencing"
        print "Skipping orthophoto"
        args.end_with = "odm_georeferencing"

    if "csString" not in jobOptions:
        parse_coordinate_system()

    if "csString" in jobOptions and "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions:
        images = []
        with open(jobOptions["jobDir"] + "/pmvs/list.rd.txt") as f:
            images = f.readlines()

        if len(images) > 0:
            with open(jobOptions["jobDir"] + "/odm_georeferencing/coordFile.txt") as f:
                for lineNumber, line in enumerate(f):
                    if lineNumber >= 2 and lineNumber - 2 < len(images):
                        tokens = line.split(' ')
                        
                        if len(tokens) >= 3:
                            x = float(tokens[0])
                            y = float(tokens[1])
                            z = float(tokens[2])
                            filename = images[lineNumber - 2]
                            
                            run("echo " + str(x + jobOptions["utmEastOffset"]) + " "                          \
                                + str(y + jobOptions["utmNorthOffset"]) + " " + str(z)                        \
                                + " | cs2cs " + jobOptions["csString"] + " +to +datum=WGS84 +proj=latlong > " \
                                + jobOptions["jobDir"] + "/odm_georeferencing/latlong.txt")
                            
                            with open(jobOptions["jobDir"] + "/odm_georeferencing/latlong.txt") as latlongFile:
                                latlongLine = latlongFile.readline()
                                tokens = latlongLine.split()
                                if len(tokens) >= 2:
                                    exifGpsInfoWritten = False

                                    lonString = tokens[0]  # Example: 83d18'16.285"W
                                    latString = tokens[1]  # Example: 41d2'11.789"N
                                    altString = ""
                                    if len(tokens) > 2:
 
                                        altString = tokens[2] # Example: 0.998

                                    tokens = re.split("[d '\"]+", lonString)
                                    if len(tokens) >= 4:
                                        lonDeg = tokens[0]
                                        lonMin = tokens[1]
                                        lonSec = tokens[2]
                                        lonSecFrac = fractions.Fraction(lonSec)
                                        lonSecNumerator = str(lonSecFrac._numerator)
                                        lonSecDenominator = str(lonSecFrac._denominator)
                                        lonRef = tokens[3]

                                        tokens = re.split("[d '\"]+", latString)
                                        if len(tokens) >= 4:
                                            latDeg = tokens[0]
                                            latMin = tokens[1]
                                            latSec = tokens[2]
                                            latSecFrac = fractions.Fraction(latSec)
                                            latSecNumerator = str(latSecFrac._numerator)
                                            latSecDenominator = str(latSecFrac._denominator)
                                            latRef = tokens[3]

                                            exivCmd = "exiv2 -q"
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitude " + latDeg + "/1 "         \
                                                + latMin + "/1 " + latSecNumerator + "/" + latSecDenominator + "\""
                                            
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLatitudeRef " + latRef + "\""
                                            
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitude " + lonDeg + "/1 "         \
                                                + lonMin + "/1 " + lonSecNumerator + "/" + lonSecDenominator + "\""
                                            
                                            exivCmd += " -M\"set Exif.GPSInfo.GPSLongitudeRef " + lonRef + "\""

                                            altNumerator = arcDenominator = 0  # BUG: arcDenominator is never used
                                            
                                            if altString:
                                                altFrac = fractions.Fraction(altString)
                                                altNumerator = str(altFrac._numerator)
                                                altDenominator = str(altFrac._denominator)
                                                exivCmd += " -M\"set Exif.GPSInfo.GPSAltitude " + altNumerator + "/" + altDenominator + "\""
                                                exivCmd += " -M\"set Exif.GPSInfo.GPSAltitudeRef 0\""

                                            exivCmd += " " + filename
                                            run(exivCmd)
                                            exifGpsInfoWritten = True

                                    if not exifGpsInfoWritten:
                                        print("    Warning: Failed setting EXIF GPS info for " \
                                              + filename + " based on " + latlongLine)

    if "epsg" in jobOptions and "utmEastOffset" in jobOptions and "utmNorthOffset" in jobOptions:
        lasCmd = "\"" + BIN_PATH + "/txt2las\" -i " + jobOptions["jobDir"] +                                         \
            "-results/option-0000_georef.ply -o " + jobOptions["jobDir"]                                             \
            + "-results/pointcloud_georef.laz -skip 30 -parse xyzRGBssss -set_scale 0.01 0.01 0.01 -set_offset "     \
            + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"]) + " 0 -translate_xyz "      \
            + str(jobOptions["utmEastOffset"]) + " " + str(jobOptions["utmNorthOffset"])                             \
            + " 0 -epsg " + str(jobOptions["epsg"])
        
        print("    Creating geo-referenced LAS file (expecting warning)...")
        
        run(lasCmd)
 

    if args['--end-with'] != "odm_georeferencing":
        odm_orthophoto()