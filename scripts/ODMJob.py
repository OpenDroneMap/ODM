class ODMJob:
    '''   ODMJob - a class for ODM Activities
    '''
    def __init__(self, inputDir, args):
        self.args = args
        self.pathDirJPGs = inputDir
        
        self.count    = 0
        self.good     = 0
        self.bad      = 0
        self.minWidth = 0.
        self.minHeight= 0
        self.maxWidth = 0
        self.maxHeight= 0
        
        self.resizeTo = 0.
        self.srcDir = CURRENT_DIR
        self.utmZone = -999
        self.utmSouth = False
        self.utmEastOffset = 0.
        self.utmNorthOffset = 0.
        
        self.jobOptions = {'resizeTo': 0, 'srcDir': CURRENT_DIR, 'utmZone': -999, 
                      'utmSouth': False, 'utmEastOffset': 0, 'utmNorthOffset': 0}
        
        self.dictJobLocations = {}   # hold our filepaths and dirs

        self.listFiles = os.listdir(CURRENT_DIR)
        self.listJPG = []
        self.listObjPhotos = []    

        
        # create obj.listJPG of all jpegs 
        for files in self.listFiles:
            (pathfn,ext) = os.path.splitext(files)
            if 'jpg' in ext.lower() :
                #print files
                self.listJPG.append(files)
            elif 'jpeg' in ext.lower() : 
                self.listJPG.append(files)
                
        print "\n  - source files - " + now()
    
        for filename in self.listJPG:
            filename = filename.rstrip('\n')
            if not filename:
                continue
            filename = CURRENT_DIR + os.sep + filename 
            print filename
            self.listObjPhotos.append( ODMPhoto( filename, self) )        

    def resize(self):
        print "\n  - preparing images - " + now()
    
        os.chdir(self.jobDir)
    
        for objPhotos in self.listObjPhotos:
            if objPhotos.isOk:
                if not os.path.isfile(objPhotos.dictStepVals["step_0_resizedImage"]):
                    if self.resizeTo != 0 and \
                       ((int(objPhotos.width) > self.resizeTo) or (objPhotos.height > self.resizeTo)):
                       
                        sys.stdout.write("    resizing " + objPhotos.strFileName +" \tto " \
                                         + objPhotos.dictStepVals["step_0_resizedImage"])
                        
                        run("convert -resize " + str(self.resizeTo) + "x" + str(self.resizeTo)\
                            +" -quality 100 \"" + self.srcDir + "/" + objPhotos.strFileName + "\" \"" \
                            + objPhotos.dictStepVals["step_0_resizedImage"] + "\"")
                    
                    else:
                        sys.stdout.write("     copying " + objPhotos.strFileName + " \tto " \
                                         + objPhotos.dictStepVals["step_0_resizedImage"])
                        shutil.copyfile(CURRENT_DIR + "/" + objPhotos.strFileName, objPhotos.dictStepVals["step_0_resizedImage"])
                else:	
                    print "     using existing " + objPhotos.strFileName + " \tto " \
                          + objPhotos.dictStepVals["step_0_resizedImage"]
    
                file_resolution = runAndReturn('jhead "' + objPhotos.dictStepVals["step_0_resizedImage"] \
                                               + '"', 'grep "Resolution"')
                match = re.search(": ([0-9]*) x ([0-9]*)", file_resolution)
                if match:
                    objPhotos.width = int(match.group(1).strip())
                    objPhotos.height = int(match.group(2).strip())
                print "\t (" + str(objPhotos.width) + " x " + str(objPhotos.height) + ")"
    