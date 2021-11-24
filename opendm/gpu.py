import os
import sys
from opendm import log
from repoze.lru import lru_cache

def gpu_disabled_by_user():
    return bool(os.environ.get('ODM_NO_GPU'))

@lru_cache(maxsize=None)
def has_gpus():
    if gpu_disabled_by_user():
        log.ODM_INFO("Disabling GPU features (ODM_NO_GPU is set)")
        return False

    try:
        import pyopencl
    except:
        return False

    try:
        platforms = pyopencl.get_platforms()
        for p in platforms:
            log.ODM_INFO("Found GPU device: %s" % p.name)

        return len(platforms) > 0
    except Exception as e:
        return False

@lru_cache(maxsize=None)
def windows_no_cuda():
    """
    Check if CUDA lib is available on Windows
    Returns true if OS is windows and CUDA is not found.
    """
    if sys.platform == 'win32':
        nvcuda_path = os.path.join(os.environ.get('SYSTEMROOT'), 'system32', 'nvcuda.dll')
        if os.path.isfile(nvcuda_path):
            return False
        else:
            log.ODM_INFO("No CUDA drivers detected, using CPU")
            return True
    else:
        return False

