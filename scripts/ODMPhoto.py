class ODMPhoto:
    """   ODMPhoto - a class for ODMPhotos
    """

    def __init__(self, inputJPG, objODMJob):
        #general purpose
        verbose = False
        # object attributes
        self.dictStepVals           = {}
        self.pathToContainingFolder = os.path.split(inputJPG)[0]
        self.strFileName            = os.path.split(inputJPG)[1]
        self.strFileNameBase        = os.path.splitext(self.strFileName)[0]
        self.strFileNameExt         = os.path.splitext(self.strFileName)[1]
        
        #start pipe for jhead
        cmdSrc = BIN_PATH + os.sep + "jhead "+ CURRENT_DIR + os.sep + self.strFileName
        srcProcess = subprocess.Popen(cmdSrc, stdout=subprocess.PIPE)

        stdout, stderr = srcProcess.communicate()
        stringOutput = stdout.decode('ascii')
        
        #listOutput is the list of params to be processed
        listOutput_ori = stringOutput.splitlines()
        listOutput = remove_values_from_list(listOutput_ori,u"")
        
        intListCount = 0
        intNumCameraAtts = len(listOutput)
        
        flagDoneList = False
        
        if verbose:  print listOutput
        
        for lines in listOutput:
            # check if we've read all atts
            intListCount += 1
            if intListCount == intNumCameraAtts: flagDoneList = True
           
            #extract and proceed            
            firstColon = lines.find(":")
            tempKey =   lines[:firstColon].strip()
            tempVal = lines[firstColon+1:].strip()
            
            if verbose:   print tempKey,tempVal
            # all them values
            if tempKey == 'File name': self.fileName = tempVal
            elif tempKey == 'File size': self.fileSize= tempVal
            elif tempKey == 'File date': self.fileDate = tempVal
            elif tempKey == 'Camera make': self.cameraMake = tempVal
            elif tempKey == 'Camera model': self.cameraModel = tempVal
            elif tempKey == 'Date/Time': self.dateTime = tempVal
            elif tempKey == 'Resolution': self.resolution = tempVal
            elif tempKey == 'Flash used': self.flashUsed = tempVal
            elif tempKey == 'Focal length': self.focalLength = tempVal
            elif tempKey == 'CCD width': self.ccdWidth = tempVal
            elif tempKey == 'Exposure time': self.exposureTime = tempVal
            elif tempKey == 'Aperture': self.aperture = tempVal
            elif tempKey == 'Focus dist.': self.focusDist = tempVal
            elif tempKey == 'ISO equiv.': self.isoEquiv= tempVal
            elif tempKey == 'Whitebalance': self.whitebalance = tempVal
            elif tempKey == 'Metering Mode': self.meteringMode = tempVal
            elif tempKey == 'GPS Latitude': self.gpsLatitude = tempVal
            elif tempKey == 'GPS Longitude': self.gpsLongitude = tempVal
            elif tempKey == 'GPS Altitude': self.gpsAltitude = tempVal
            elif tempKey == 'JPEG Quality': self.jpgQuality = tempVal	
            #  better object attribute names; keep old for compatability
            #  shallow references point to same stack space
            self.fullPathAndName = self.fileName

            # attribute 'id' set to more specific of the maker or model
            try:
                if self.cameraMake:
                    self.make = self.cameraMake
                    self.id = self.cameraMake
            except:  pass

            
            try:
                if self.cameraModel:
                    self.model = self.cameraModel
                    self.id = self.cameraModel
            except:  pass
            
            # parse resolution field 
            try:
                match = re.search("([0-9]*) x ([0-9]*)",self.resolution)
                if match:
                    self.width  = int(match.group(1).strip())
                    self.height = int(match.group(2).strip())
            except:  pass
            
            #parse force-focal
            try:
                if not '--force-focal' in args:
                    match = re.search(":[\ ]*([0-9\.]*)mm", self.focalLength)
                    if match:
                        self.focal = float((match.group()[1:-2]).strip())
                else:
                    self.focal = args['--force-focal']
            except: pass

            #parse force-ccd
            if 'ccd' in lines.lower():
                if not '--force-ccd' in args:
                    try:
                        floats = extractFloat(self.ccdWidth)
                        self.ccd = floats[0]
                    except:
                        try:
                            self.ccd = float(ccdWidths[self.id])
                        except: pass
                else:
                    self.ccd = args['--force-ccd']


            if verbose:  print intListCount
            
            if flagDoneList:
                try:
                    if self.width > self.height:
                        self.focalpx = self.width * (self.focal / self.ccd)
                    else:
                        self.focalpx = self.height * (self.focal / self.ccd)
        
                    self.isOk = True
                    objODMJob.good += 1
        
                    print "     using " + self.fileName + "     dimensions: " + \
                          str(self.width) + "x" + str(self.height)\
                          + " | focal: " + str(self.focal) \
                          + "mm | ccd: " + str(self.ccd) + "mm"
                    
                except:
                    self.isOk = False
                    objODMJob.bad += 1
    
                    try:
                        print "\n    no CCD width or focal length found for "\
                              + self.fileName+ " - camera: \"" + self.id+ "\""
                    except:
                        print "\n    no CCD width or focal length found"
            
                #either way increment total count
                objODMJob.count += 1
    
                #populate & update max/mins
                
                if objODMJob.minWidth == 0:
                    objODMJob.minWidth = self.width
                               
                if objODMJob.minHeight == 0:
                    objODMJob.minHeight = self.height

                if objODMJob.minWidth < self.width:
                    objODMJob.minWidth = self.minWidth
                else:
                    objODMJob.minWidth = self.width
               
                if objODMJob.minHeight < self.height:
                    objODMJob.minHeight = objODMJob.minHeight
                else:
                    objODMJob.minHeight = self.height

                if objODMJob.maxWidth > self.width:
                    objODMJob.maxWidth = objODMJob.maxWidth
                else:
                    objODMJob.maxWidth = self.width

                if objODMJob.maxHeight > self.height:
                    objODMJob.maxHeight = objODMJob.maxHeight
                else:
                    objODMJob.maxHeight = self.height
    
