import os
import shutil, errno
import json

def absolute_path_file(path_file):
    return os.path.abspath(path_file)


def extract_path_from_file(file):
    path_file = os.path.abspath(os.path.dirname(file))
    path, file = os.path.split(path_file)
    return path


def join_paths(*args):
    return os.path.join(*args)


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

def rename_file(src, dst):
    try:
        os.rename(src, dst)
        return True
    except OSError as e:
        if e.errno == errno.ENOENT:
            return False
        else:
            raise


# find a file in the root directory
def find(filename, folder):
    for root, dirs, files in os.walk(folder):
        return '/'.join((root, filename)) if filename in files else None


def related_file_path(input_file_path, prefix="", postfix="", replace_base=None):
    """
    For example: related_file_path("/path/to/file.ext", "a.", ".b")
     --> "/path/to/a.file.b.ext"
    """
    path, filename = os.path.split(input_file_path)

    # path = path/to
    # filename = file.ext

    basename, ext = os.path.splitext(filename)
    # basename = file
    # ext = .ext

    if replace_base is not None:
        basename = replace_base

    return os.path.join(path, "{}{}{}{}".format(prefix, basename, postfix, ext))

def path_or_json_string_to_dict(string):
    if string == "":
        return {}

    if string.startswith("[") or string.startswith("{"):
        try:
            return json.loads(string)
        except:
            raise ValueError("{0} is not a valid JSON string.".format(string))
    elif file_exists(string):
        try:
            with open(string, 'r') as f:
                return json.loads(f.read())
        except:
            raise ValueError("{0} is not a valid JSON file.".format(string))
    else:
        raise ValueError("{0} is not a valid JSON file or string.".format(string))
