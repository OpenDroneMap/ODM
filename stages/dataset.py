import os
import json

from opendm import context
from opendm import io
from opendm import types
from opendm import log
from opendm import system
from opendm import location
from shutil import copyfile

def save_images_database(photos, database_file):
    with open(database_file, 'w') as f:
        f.write(json.dumps(map(lambda p: p.__dict__, photos)))
    
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
        # Load tree
        tree = types.ODM_Tree(args.project_path, args.images, args.gcp)
        outputs['tree'] = tree

        if args.time and io.file_exists(tree.benchmarking):
            # Delete the previously made file
            os.remove(tree.benchmarking)
            with open(tree.benchmarking, 'a') as b:
                b.write('ODM Benchmarking file created %s\nNumber of Cores: %s\n\n' % (system.now(), context.num_cores))
    
        # check if the extension is supported
        def supported_extension(file_name):
            (pathfn, ext) = os.path.splitext(file_name)
            return ext.lower() in context.supported_extensions

        # Get supported images from dir
        def get_images(in_dir):
            # filter images for its extension type
            log.ODM_DEBUG(in_dir)
            return [f for f in io.get_files_list(in_dir) if supported_extension(f)]

        # get images directory
        input_dir = tree.input_images
        images_dir = tree.dataset_raw

        if not io.dir_exists(images_dir):
            log.ODM_INFO("Project directory %s doesn't exist. Creating it now. " % images_dir)
            system.mkdir_p(images_dir)
            copied = [copyfile(io.join_paths(input_dir, f), io.join_paths(images_dir, f)) for f in get_images(input_dir)]

        # define paths and create working directories
        system.mkdir_p(tree.odm_georeferencing)
        if not args.use_3dmesh: system.mkdir_p(tree.odm_25dgeoreferencing)

        log.ODM_DEBUG('Loading dataset from: %s' % images_dir)

        # check if we rerun cell or not
        images_database_file = io.join_paths(tree.root_path, 'images.json')
        if not io.file_exists(images_database_file) or self.rerun():
            files = get_images(images_dir)
            if files:
                # create ODMPhoto list
                path_files = [io.join_paths(images_dir, f) for f in files]

                photos = []
                with open(tree.dataset_list, 'w') as dataset_list:
                    for f in path_files:
                        photos += [types.ODM_Photo(f)]
                        dataset_list.write(photos[-1].filename + '\n')

                # Save image database for faster restart
                save_images_database(photos, images_database_file)
            else:
                log.ODM_ERROR('Not enough supported images in %s' % images_dir)
                exit(1)
        else:
            # We have an images database, just load it
            photos = load_images_database(images_database_file)

        log.ODM_INFO('Found %s usable images' % len(photos))

        # append photos to cell output
        if not self.params.get('proj'):
            if tree.odm_georeferencing_gcp:
                outputs['reconstruction'] = types.ODM_Reconstruction(photos, coords_file=tree.odm_georeferencing_gcp)
            else:
                # Generate UTM from images
                try:
                    if not io.file_exists(tree.odm_georeferencing_coords) or self.rerun():
                        location.extract_utm_coords(photos, tree.dataset_raw, tree.odm_georeferencing_coords)
                    else:
                        log.ODM_INFO("Coordinates file already exist: %s" % tree.odm_georeferencing_coords)
                except:
                    log.ODM_WARNING('Could not generate coordinates file. '
                                    'Ignore if there is a GCP file')

                outputs['reconstruction'] = types.ODM_Reconstruction(photos, coords_file=tree.odm_georeferencing_coords)
        else:
            outputs['reconstruction'] = types.ODM_Reconstruction(photos, projstring=self.params.get('proj'))

        # Save proj to file for future use (unless this 
        # dataset is not georeferenced)
        if outputs['reconstruction'].projection:
            with open(io.join_paths(tree.odm_georeferencing, tree.odm_georeferencing_proj), 'w') as f:
                f.write(outputs['reconstruction'].projection.srs)

