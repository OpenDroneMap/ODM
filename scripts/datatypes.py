import os
import subprocess

import log
import dataset
import system
import tasks

class ODMApp:
    '''   ODMApp - a class for ODM Activities
    '''
    def __init__(self, args):
        # Internal app config
        self.args = args
        self.images_dir = os.path.abspath(args['images_src'])
        # Initialize odm photos
        self.photos = []
        # Task manager
        # configure and schedule tasks
        self.task_manager = tasks.ODMTaskManager(self)

    # Run all tasks given an starting point
    def run_all(self, initial_task, final_task):

        # found ids for tasks
        t_dict = tasks.tasks_dict
        initial_task_id = [k for k in t_dict if t_dict[k]==initial_task][0]
        final_task_id = [k for k in t_dict if t_dict[k]==final_task][0]

        if initial_task_id > final_task_id:
            log.ODM_ERROR('Initial task must be lower than final')
            return

        # setup task manager
        self.task_manager.initial_task_id = int(initial_task_id)
        self.task_manager.final_task_id = int(final_task_id)
        # run defined pipeline
        self.task_manager.run_tasks()


class ODMPhoto:
    """   ODMPhoto - a class for ODMPhotos
    """
    def __init__(self, path, args):
        #  general purpose
        self.path = path
        self.path_file = None
        # current attibutes
        self.file_name = None
        self.width = None
        self.height = None
        # other attributes
        self.file_size = None
        self.file_date = None
        self.camera_make = None
        self.camera_model = None
        self.date_time = None
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
        self.exposure_mode = None
        self.exposure_bias = None
        self.orientation = None
        self.gps_latitude = None
        self.gps_longitude = None
        self.gps_altitude = None
        self.jpg_quality = None

        # parse values
        self.parse_jhead_values(self.path, args)

        # compute focal lenght into pixels
        self.compute_focal_length()

        # TODO(edgar): compute global min/max
        # def compute_min_max()
            # min_width = min(min_width, width)
            # max_width = max(max_width, width)
            # min_height = min(min_height, heigth)
            # max_height = max(max_height, height)

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
        if self.focal_length is not None and self.ccd_width is not None:
            # compute the focal lenth in pixels
            if self.width > self.height:
                self.focal_length_px = \
                    self.width * (self.focal_length / self.ccd_width)
            else:
                self.focal_length_px = \
                    self.height * (self.focal_length / self.ccd_width)

            log.ODM_DEBUG('Loaded %s | dimensions: %s x %s | focal: %smm | ccd: %smm' % \
                (self.file_name, self.width, self.height, self.focal_length, self.ccd_width))
        else:
            log.ODM_WARNING('No CCD width or focal length found for image file: \n' + 
                self.file_name + ' camera: \"' + self.camera_model)

    def parse_jhead_values(self, _path, args):

         # load ccd_widths from file
        ccd_widths = system.get_ccd_widths()

        # start pipe for jhead
        src_process = subprocess.Popen(['jhead', _path], 
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
            if key == 'File name': 
                self.path_file = val
                self.file_name = dataset.extract_file_from_path_file(val)
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
                if args.get('force_focal') is not None:
                    self.focal_length = args['force_focal']
                else:
                    self.focal_length = float(val.split(' ')[0][:-2])
            
            # parse force-ccd
            elif key == 'CCD width': 
                if args.get('force_ccd') is not None:
                    self.ccd_width = args['force_ccd']
                else:
                    self.ccd_width = float(val[:-2])

            elif key == 'Exposure time': self.exposure_time = val
            elif key == 'Aperture': self.aperture = val
            elif key == 'Focus dist.': self.focus_dist = val
            elif key == 'ISO equiv.': self.iso_equiv = val
            elif key == 'Whitebalance': self.white_balance = val
            elif key == 'Light Source': self.light_source = val
            elif key == 'Metering Mode': self.metering_mode = val
            elif key == 'Exposure': self.exposure = val
            elif key == 'Exposure Mode': self.exposure_mode = val
            elif key == 'Exposure bias': self.exposure_bias = val
            elif key == 'Orientation': self.orientation = val
            elif key == 'GPS Latitude': self.gps_latitude = val
            elif key == 'GPS Longitude': self.gps_longitude = val
            elif key == 'GPS Altitude': self.gps_altitude = val
            elif key == 'JPEG Quality': self.jpg_quality = val
            else: 
                log.ODM_WARNING('Unknown key: %s' % key)
            
        # find ccd_width from file if needed
        if self.ccd_width is None:
            key = [x for x in ccd_widths.keys() if self.camera_model in x][0]
            if key is not None:
                self.ccd_width = float(ccd_widths[key])
            else:
                log.ODM_ERROR('Could not find ccd_width in file')
   