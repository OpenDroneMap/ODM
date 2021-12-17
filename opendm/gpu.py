import os
import sys
import shutil
from opendm import log
from repoze.lru import lru_cache

def gpu_disabled_by_user():
    return bool(os.environ.get('ODM_NO_GPU'))

@lru_cache(maxsize=None)
def has_popsift_and_can_handle_texsize(width, height):
    try:
        from opensfm import pypopsift
        fits = pypopsift.fits_texture(width, height)
        if not fits:
            log.ODM_WARNING("Image size (%sx%spx) would not fit in GPU memory, falling back to CPU" % (width, height))
        return fits
    except (ModuleNotFoundError, ImportError):
        return False
    except Exception as e:
        log.ODM_WARNING(str(e))
        return False

@lru_cache(maxsize=None)
def has_gpu():
    if gpu_disabled_by_user():
        log.ODM_INFO("Disabling GPU features (ODM_NO_GPU is set)")
        return False

    if sys.platform == 'win32':
        nvcuda_path = os.path.join(os.environ.get('SYSTEMROOT'), 'system32', 'nvcuda.dll')
        if os.path.isfile(nvcuda_path):
            return True
        else:
            log.ODM_INFO("No CUDA drivers detected, using CPU")
            return False
    else:
        if shutil.which('nvidia-smi') is not None:
            return True
        else:
            log.ODM_INFO("nvidia-smi not found in PATH, using CPU")
            return False
