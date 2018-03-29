import os
import shutil, errno


def get_files_list(path_dir):
    return os.listdir(path_dir)


def absolute_path_file(path_file):
    return os.path.abspath(path_file)


def extract_file_from_path_file(path_file):
    path, file = os.path.split(path_file)
    return file


def extract_path_from_file(file):
    path_file = os.path.abspath(os.path.dirname(file))
    path, file = os.path.split(path_file)
    return path


def join_paths(path1, path2):
    return os.path.join(path1, path2)


def file_exists(path_file):
    return os.path.isfile(path_file)


def dir_exists(dirname):
    return os.path.isdir(dirname)


def copy(src, dst):
    try: 
        shutil.copytree(src, dst)
    except OSError as e:
        if e.errno == errno.ENOTDIR:
            shutil.copy(src, dst)
        else: raise


# find a file in the root directory
def find(filename, folder):
    for root, dirs, files in os.walk(folder):
        return '/'.join((root, filename)) if filename in files else None
