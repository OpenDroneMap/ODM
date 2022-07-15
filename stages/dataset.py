import os
import json

from opendm import context
from opendm import io
from opendm import types
from opendm.photo import PhotoCorruptedException
from opendm import log
from opendm import system
from opendm.geo import GeoFile
from shutil import copyfile
from opendm import progress
from opendm import boundary
from opendm import ai
from opendm.skyremoval.skyfilter import SkyFilter
from opendm.concurrency import parallel_map

def save_images_database(photos, database_file):
    with open(database_file, 'w') as f:
        f.write(json.dumps([p.__dict__ for p in photos]))
    
    log.ODM_INFO("Wrote images database: %s" % database_file)

def load_images_database(database_file):
    # Empty is used to create types.ODM_Photo class
    # instances without calling __init__
    class Empty:
        pass

    result = []

    log.ODM_INFO("Loading images database: %s" % database_file)

    with open(database_file, 'r') as f:
        photos_json = json.load(f)
        for photo_json in photos_json:
            p = Empty()
            for k in photo_json:
                setattr(p, k, photo_json[k])
            p.__class__ = types.ODM_Photo
            result.append(p)

    return result

class ODMLoadDatasetStage(types.ODM_Stage):
    def process(self, args, outputs):
        outputs['start_time'] = system.now_raw()
        tree = types.ODM_Tree(args.project_path, args.gcp, args.geo)
        outputs['tree'] = tree

        if args.time and io.file_exists(tree.benchmarking):
            # Delete the previously made file
            os.remove(tree.benchmarking)
            with open(tree.benchmarking, 'a') as b:
                b.write('ODM Benchmarking file created %s\nNumber of Cores: %s\n\n' % (system.now(), context.num_cores))
    
        # check if the image filename is supported
        def valid_image_filename(filename):
            (pathfn, ext) = os.path.splitext(filename)
            return ext.lower() in context.supported_extensions and pathfn[-5:] != "_mask"

        # Get supported images from dir
        def get_images(in_dir):
            log.ODM_DEBUG(in_dir)
            entries = os.listdir(in_dir)
            valid, rejects = [], []
            for f in entries:
                if valid_image_filename(f):
                    valid.append(f)
                else:
                    rejects.append(f)
            return valid, rejects

        def find_mask(photo_path, masks):
            (pathfn, ext) = os.path.splitext(os.path.basename(photo_path))
            k = "{}_mask".format(pathfn)
            
            mask = masks.get(k)
            if mask:
                # Spaces are not supported due to OpenSfM's mask_list.txt format reqs
                if not " " in mask:
                    return mask
                else:
                    log.ODM_WARNING("Image mask {} has a space. Spaces are currently not supported for image masks.".format(mask))

        # get images directory
        images_dir = tree.dataset_raw

        # define paths and create working directories
        system.mkdir_p(tree.odm_georeferencing)

        log.ODM_INFO('Loading dataset from: %s' % images_dir)

        # check if we rerun cell or not
        images_database_file = os.path.join(tree.root_path, 'images.json')
        if not io.file_exists(images_database_file) or self.rerun():
            if not os.path.exists(images_dir):
                raise system.ExitException("There are no images in %s! Make sure that your project path and dataset name is correct. The current is set to: %s" % (images_dir, args.project_path))

            files, rejects = get_images(images_dir)
            if files:
                # create ODMPhoto list
                path_files = [os.path.join(images_dir, f) for f in files]

                # Lookup table for masks
                masks = {}
                for r in rejects:
                    (p, ext) = os.path.splitext(r)
                    if p[-5:] == "_mask" and ext.lower() in context.supported_extensions:
                        masks[p] = r

                photos = []
                with open(tree.dataset_list, 'w') as dataset_list:
                    log.ODM_INFO("Loading %s images" % len(path_files))
                    for f in path_files:
                        try:
                            p = types.ODM_Photo(f)
                            p.set_mask(find_mask(f, masks))
                            photos.append(p)
                            dataset_list.write(photos[-1].filename + '\n')
                        except PhotoCorruptedException:
                            log.ODM_WARNING("%s seems corrupted and will not be used" % os.path.basename(f))

                # Check if a geo file is available
                if tree.odm_geo_file is not None and os.path.isfile(tree.odm_geo_file):
                    log.ODM_INFO("Found image geolocation file")
                    gf = GeoFile(tree.odm_geo_file)
                    updated = 0
                    for p in photos:
                        entry = gf.get_entry(p.filename)
                        if entry:
                            p.update_with_geo_entry(entry)
                            p.compute_opk()
                            updated += 1
                    log.ODM_INFO("Updated %s image positions" % updated)

                # GPSDOP override if we have GPS accuracy information (such as RTK)
                if 'gps_accuracy_is_set' in args:
                    log.ODM_INFO("Forcing GPS DOP to %s for all images" % args.gps_accuracy)

                    for p in photos:
                        p.override_gps_dop(args.gps_accuracy)
                
                # Override projection type
                if args.camera_lens != "auto":
                    log.ODM_INFO("Setting camera lens to %s for all images" % args.camera_lens)

                    for p in photos:
                        p.override_camera_projection(args.camera_lens)

                # Automatic sky removal
                if args.sky_removal:
                    # For each image that :
                    #  - Doesn't already have a mask, AND
                    #  - Is not nadir (or if orientation info is missing), AND
                    #  - There are no spaces in the image filename (OpenSfM requirement)
                    # Automatically generate a sky mask
                    
                    # Generate list of sky images
                    sky_images = []
                    for p in photos:
                        if p.mask is None and (p.pitch is None or (abs(p.pitch) > 20)) and (not " " in p.filename):
                            sky_images.append({'file': os.path.join(images_dir, p.filename), 'p': p})

                    if len(sky_images) > 0:
                        log.ODM_INFO("Automatically generating sky masks for %s images" % len(sky_images))
                        model = ai.get_model("skyremoval", "https://github.com/OpenDroneMap/SkyRemoval/releases/download/v1.0.5/model.zip", "v1.0.5")
                        if model is not None:
                            sf = SkyFilter(model=model)

                            def parallel_sky_filter(item):
                                try:
                                    mask_file = sf.run_img(item['file'], images_dir)

                                    # Check and set
                                    if mask_file is not None and os.path.isfile(mask_file):
                                        item['p'].set_mask(os.path.basename(mask_file))
                                        log.ODM_INFO("Wrote %s" % os.path.basename(mask_file))
                                    else:
                                        log.ODM_WARNING("Cannot generate mask for %s" % img)
                                except Exception as e:
                                    log.ODM_WARNING("Cannot generate mask for %s: %s" % (img, str(e)))

                            parallel_map(parallel_sky_filter, sky_images, max_workers=args.max_concurrency)

                            log.ODM_INFO("Sky masks generation completed!")
                        else:
                            log.ODM_WARNING("Cannot load AI model (you might need to be connected to the internet?)")
                    else:
                        log.ODM_INFO("No sky masks will be generated (masks already provided, or images are nadir)")

                # End sky removal

                # Save image database for faster restart
                save_images_database(photos, images_database_file)
            else:
                raise system.ExitException('Not enough supported images in %s' % images_dir)
        else:
            # We have an images database, just load it
            photos = load_images_database(images_database_file)

        log.ODM_INFO('Found %s usable images' % len(photos))
        log.logger.log_json_images(len(photos))

        # Create reconstruction object
        reconstruction = types.ODM_Reconstruction(photos)
        
        if tree.odm_georeferencing_gcp and not args.use_exif:
            reconstruction.georeference_with_gcp(tree.odm_georeferencing_gcp,
                                                 tree.odm_georeferencing_coords,
                                                 tree.odm_georeferencing_gcp_utm,
                                                 tree.odm_georeferencing_model_txt_geo,
                                                 rerun=self.rerun())
        else:
            reconstruction.georeference_with_gps(tree.dataset_raw, 
                                                 tree.odm_georeferencing_coords, 
                                                 tree.odm_georeferencing_model_txt_geo,
                                                 rerun=self.rerun())
        
        reconstruction.save_proj_srs(os.path.join(tree.odm_georeferencing, tree.odm_georeferencing_proj))
        outputs['reconstruction'] = reconstruction

        # Try to load boundaries
        if args.boundary:
            if reconstruction.is_georeferenced():
                outputs['boundary'] = boundary.load_boundary(args.boundary, reconstruction.get_proj_srs())
            else:
                args.boundary = None
                log.ODM_WARNING("Reconstruction is not georeferenced, but boundary file provided (will ignore boundary file)")

        # If sfm-algorithm is triangulation, check if photos have OPK
        if args.sfm_algorithm == 'triangulation':
            for p in photos:
                if not p.has_opk():
                    log.ODM_WARNING("No omega/phi/kappa angles found in input photos (%s), switching sfm-algorithm to incremental" % p.filename)
                    args.sfm_algorithm = 'incremental'
                    break
