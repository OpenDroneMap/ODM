import os
import subprocess

import log
import context
from tasks import ODMTaskManager

class ODMApp:
    '''   ODMJob - a class for ODM Activities
    '''
    def __init__(self, images_dir, args=None):
        # Internal app config
        self.args = args
        self.images_dir = os.path.abspath(images_dir)

        # Initialize odm photos
        self.photos = self.init_photos(self.images_dir, self.args)
        
        # Task manager
        # configure and schedule tasks
        self.task_manager = ODMTaskManager()

    # Run all tasks given an starting point
    def run(self, initial_task_id):

        self.task_manager.initial_task_id = initial_task_id
        self.task_manager.run_tasks()


    def init_photos(self, images_dir, args):

        # check if the extension is sopported
        def supported_extension(file_name):
            (pathfn, ext) = os.path.splitext(file_name)
            return ext.lower() in context.supported_extensions

        # find files in the given directory
        files = os.listdir(images_dir)

        # filter images for its extension type
        # by now only 'jpg' and 'jpeg are supported
        files = [f for f in files if supported_extension(f)]

        photos = []

        # create ODMPhoto list
        for f in files:
            file_name = os.path.join(images_dir, f)
            photos.append(ODMPhoto(file_name, args))

        return photos


class ODMPhoto:
    """   ODMPhoto - a class for ODMPhotos
    """
    def __init__(self, file_name, args):
        #  general purpose
        self.file_name = file_name

        # photo ideal attibutes
        self.file_size = None
        self.file_date = None
        self.camera_make = None
        self.camera_model = None
        self.date_time = None
        self.width = None
        self.height = None
        self.resolution = None
        self.flash_used = None
        self.focal_length = None
        self.focal_length_px = None
        self.ccd_width = None
        self.exposure_time = None
        self.aperture = None
        self.focus_distance = None
        self.iso_equiv = None
        self.white_balance = None
        self.light_source = None
        self.metering_mode = None
        self.exposure = None
        self.gps_latitude = None
        self.gps_longitude = None
        self.gps_altitude = None
        self.jpg_quality = None

        # parse values
        self.parse_jhead_values(args)

        # compute focal lenght into pixels
        self.compute_focal_length()

        # TODO(edgar): compute global min/max
        # def compute_min_max()
            ## populate & update max/mins
#                    
            #if objODMJob.minWidth == 0:
                #objODMJob.minWidth = self.width
#                           
            #if objODMJob.minHeight == 0:
                #objODMJob.minHeight = self.height
#
            #if objODMJob.minWidth < self.width:
                #objODMJob.minWidth = self.minWidth
            #else:
                #objODMJob.minWidth = self.width
#           
            #if objODMJob.minHeight < self.height:
                #objODMJob.minHeight = objODMJob.minHeight
            #else:
                #objODMJob.minHeight = self.height
#
            #if objODMJob.maxWidth > self.width:
                #objODMJob.maxWidth = objODMJob.maxWidth
            #else:
                #objODMJob.maxWidth = self.width
#
            #if objODMJob.maxHeight > self.height:
                #objODMJob.maxHeight = objODMJob.maxHeight
            #else:
                #objODMJob.maxHeight = self.height   

    def compute_focal_length(self):

        if self.width is not None and self.height is not None and \
           self.focal_length is not None and self.ccd_width is not None:
            
            if self.width > self.height:
                self.focal_length_px = \
                    self.width * (self.focal_length / self.ccd_width)
            else:
                self.focal_length_px = \
                    self.height * (self.focal_length / self.ccd_width)

            log.ODM_INFO('Using %s dimensions: %s x %s | focal: %smm | ccd: %smm', \
                (self.file_name, self.width, self.height, self.focal_length, self.ccd_width))
        else:
            log.ODM_WARNING('No CCD width or focal length found for image file: \n' + self.file_name \
                + ' camera: \"' + self.camera_model)

    def parse_jhead_values(self, args):

        # start pipe for jhead
        src_process = subprocess.Popen(['jhead', self.file_name], 
                                       stdout=subprocess.PIPE)
        std_out, std_err = src_process.communicate()

        # split lines since the output has not a standard format
        std_out = std_out.decode('ascii')
        std_out = std_out.splitlines()
       
        # loop overl lines
        for line in std_out[:-1]:
            # split line in two parts and catch keys and values
            key, val = [x.strip() for x in line.split(':')][:2]

            # parse values to attributes
            if key == 'File name': self.file_name = val
            elif key == 'File size': self.file_size = val
            elif key == 'File date': self.file_date = val
            elif key == 'Camera make': self.camera_make = val
            elif key == 'Camera model': self.camera_model = val
            elif key == 'Date/Time': self.date_time = val
            
            # parse resolution field 
            elif key == 'Resolution':
                width, height = val.split(' x ')[:2]
                self.width = int(width)
                self.height = int(height)
            
            # parse resolution field 
            elif key == 'Flash used': 
                self.flash_used = bool(val)

            # parse force-focal
            elif key == 'Focal length':
                if args is None or not '--force-focal' in args:
                    self.focal_length = float(val.split(' ')[0][:-2])
                else:
                    self.focal_length = args['--force-focal']
            
            # parse force-ccd
            elif key == 'CCD width': 
                if args is None or not '--force-ccd' in args:
                    self.ccd_width = float(val[:-2])
                else:
                    self.ccd_width = args['--force-ccd']

            elif key == 'Exposure time': self.exposure_time = val
            elif key == 'Aperture': self.aperture = val
            elif key == 'Focus dist.': self.focus_dist = val
            elif key == 'ISO equiv.': self.iso_equiv = val
            elif key == 'Whitebalance': self.white_balance = val
            elif key == 'Light Source': self.light_source = val
            elif key == 'Metering Mode': self.metering_mode = val
            elif key == 'Exposure': self.exposure = val
            elif key == 'GPS Latitude': self.gps_latitude = val
            elif key == 'GPS Longitude': self.gps_longitude = val
            elif key == 'GPS Altitude': self.gps_altitude = val
            elif key == 'JPEG Quality': self.jpg_quality = val
            else: 
                log.ODM_ERROR('Unknown key: %s' % key)