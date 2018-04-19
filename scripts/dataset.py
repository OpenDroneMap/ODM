import os
import ecto

from opendm import context
from opendm import io
from opendm import types
from opendm import log
from opendm import system
from shutil import copyfile


def make_odm_photo(force_focal, force_ccd, path_file):
    return types.ODM_Photo(path_file,
                           force_focal,
                           force_ccd)


class ODMLoadDatasetCell(ecto.Cell):

    def declare_params(self, params):
        params.declare("force_focal", 'Override the focal length information for the '
                       'images', None)
        params.declare("force_ccd", 'Override the ccd width information for the '
                       'images', None)
        params.declare("verbose", 'indicate verbosity', False)
        params.declare("proj", 'Geographic projection', None)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        outputs.declare("reconstruction", "ODMReconstruction", [])
        inputs.declare("args", "The application arguments.", {})

    def process(self, inputs, outputs):
        # check if the extension is supported
        def supported_extension(file_name):
            (pathfn, ext) = os.path.splitext(file_name)
            return ext.lower() in context.supported_extensions

        # Get supported images from dir
        def get_images(in_dir):
            # filter images for its extension type
            log.ODM_DEBUG(in_dir)
            return [f for f in io.get_files_list(in_dir) if supported_extension(f)]

        log.ODM_INFO('Running ODM Load Dataset Cell')

        # get inputs
        tree = self.inputs.tree
        args = self.inputs.args

        # get images directory
        input_dir = tree.input_images
        images_dir = tree.dataset_raw

        if not io.dir_exists(images_dir):
            log.ODM_INFO("Project directory %s doesn't exist. Creating it now. " % images_dir)
            system.mkdir_p(images_dir)
            copied = [copyfile(io.join_paths(input_dir, f), io.join_paths(images_dir, f)) for f in get_images(input_dir)]

        # define paths and create working directories
        system.mkdir_p(tree.odm_georeferencing)
        if args.use_25dmesh: system.mkdir_p(tree.odm_25dgeoreferencing)

        log.ODM_DEBUG('Loading dataset from: %s' % images_dir)

        files = get_images(images_dir)

        if files:
            # create ODMPhoto list
            path_files = [io.join_paths(images_dir, f) for f in files]

            photos = []
            with open(tree.dataset_list, 'w') as dataset_list:
                for files in path_files:
                    photos += [make_odm_photo(self.params.force_focal, self.params.force_ccd, files)]
                    dataset_list.write(photos[-1].filename + '\n')

            log.ODM_INFO('Found %s usable images' % len(photos))
        else:
            log.ODM_ERROR('Not enough supported images in %s' % images_dir)
            return ecto.QUIT

        # append photos to cell output
        if not self.params.proj:
            if tree.odm_georeferencing_gcp:
                outputs.reconstruction = types.ODM_Reconstruction(photos, coords_file=tree.odm_georeferencing_gcp)
            else:
                verbose = '-verbose' if self.params.verbose else ''
                # Generate UTM from images
                # odm_georeference definitions
                kwargs = {
                    'bin': context.odm_modules_path,
                    'imgs': tree.dataset_raw,
                    'imgs_list': tree.dataset_list,
                    'coords': tree.odm_georeferencing_coords,
                    'log': tree.odm_georeferencing_utm_log,
                    'verbose': verbose
                }

                # run UTM extraction binary
                extract_utm = system.run_and_return('{bin}/odm_extract_utm -imagesPath {imgs}/ '
                                                    '-imageListFile {imgs_list} -outputCoordFile {coords} {verbose} '
                                                    '-logFile {log}'.format(**kwargs))

                if extract_utm != '':
                    log.ODM_WARNING('Could not generate coordinates file. '
                                    'Ignore if there is a GCP file. Error: %s'
                                    % extract_utm)

                outputs.reconstruction = types.ODM_Reconstruction(photos, coords_file=tree.odm_georeferencing_coords)
        else:
            outputs.reconstruction = types.ODM_Reconstruction(photos, projstring=self.params.proj)

        # Save proj to file for future use (unless this 
        # dataset is not georeferenced)
        if outputs.reconstruction.projection:
            with open(io.join_paths(tree.odm_georeferencing, tree.odm_georeferencing_proj), 'w') as f:
                f.write(outputs.reconstruction.projection.srs)

        log.ODM_INFO('Running ODM Load Dataset Cell - Finished')
        return ecto.OK if args.end_with != 'dataset' else ecto.QUIT
