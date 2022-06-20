import os
import sys
import shutil
import ctypes
from opendm import log
from repoze.lru import lru_cache

def gpu_disabled_by_user_env():
    return bool(os.environ.get('ODM_NO_GPU'))

@lru_cache(maxsize=None)
def has_popsift_and_can_handle_texsize(width, height):
    # We first check that we have the required compute capabilities
    # As we do not support compute capabilities less than 3.5
    try:
        compute_major, compute_minor = get_cuda_compute_version(0)
        if compute_major < 3 or (compute_major == 3 and compute_minor < 5):
            # Not supported
            log.ODM_WARNING("CUDA compute platform is not supported (detected: %s.%s but we need at least 3.5)" % (compute_major, compute_minor))
            return False
    except Exception as e:
        log.ODM_WARNING("Cannot use GPU for feature extraction: %s" % str(e))
        return False

    try:
        from opensfm import pypopsift
        fits = pypopsift.fits_texture(int(width * 1.02), int(height * 1.02))
        if not fits:
            log.ODM_WARNING("Image size (%sx%spx) would not fit in GPU memory, falling back to CPU" % (width, height))
        return fits
    except (ModuleNotFoundError, ImportError):
        return False
    except Exception as e:
        log.ODM_WARNING(str(e))
        return False

@lru_cache(maxsize=None)
def get_cuda_compute_version(device_id = 0):
    cuda_lib = "libcuda.so"
    if sys.platform == 'win32':
        cuda_lib = os.path.join(os.environ.get('SYSTEMROOT'), 'system32', 'nvcuda.dll')
        if not os.path.isfile(cuda_lib):
            cuda_lib = "nvcuda.dll"

    nvcuda = ctypes.cdll.LoadLibrary(cuda_lib)

    nvcuda.cuInit.argtypes = (ctypes.c_uint32, )
    nvcuda.cuInit.restypes = (ctypes.c_int32)

    if nvcuda.cuInit(0) != 0:
        raise Exception("Cannot initialize CUDA")

    nvcuda.cuDeviceGetCount.argtypes = (ctypes.POINTER(ctypes.c_int32), )
    nvcuda.cuDeviceGetCount.restypes = (ctypes.c_int32)
    
    device_count = ctypes.c_int32()
    if nvcuda.cuDeviceGetCount(ctypes.byref(device_count)) != 0:
        raise Exception("Cannot get device count")

    if device_count.value == 0:
        raise Exception("No devices")

    nvcuda.cuDeviceComputeCapability.argtypes = (ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32), ctypes.c_int32)
    nvcuda.cuDeviceComputeCapability.restypes = (ctypes.c_int32)
    compute_major = ctypes.c_int32()
    compute_minor = ctypes.c_int32()

    if nvcuda.cuDeviceComputeCapability(ctypes.byref(compute_major), ctypes.byref(compute_minor), device_id) != 0:
        raise Exception("Cannot get CUDA compute version")

    return (compute_major.value, compute_minor.value)

def has_gpu(args):
    if gpu_disabled_by_user_env():
        log.ODM_INFO("Disabling GPU features (ODM_NO_GPU is set)")
        return False
    if args.no_gpu:
        log.ODM_INFO("Disabling GPU features (--no-gpu is set)")
        return False

    if sys.platform == 'win32':
        nvcuda_path = os.path.join(os.environ.get('SYSTEMROOT'), 'system32', 'nvcuda.dll')
        if os.path.isfile(nvcuda_path):
            log.ODM_INFO("CUDA drivers detected")
            return True
        else:
            log.ODM_INFO("No CUDA drivers detected, using CPU")
            return False
    else:
        if shutil.which('nvidia-smi') is not None:
            log.ODM_INFO("nvidia-smi detected")
            return True
        else:
            log.ODM_INFO("nvidia-smi not found in PATH, using CPU")
            return False
