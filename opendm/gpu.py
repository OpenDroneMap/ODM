import os
from opendm import log
from repoze.lru import lru_cache

@lru_cache(maxsize=None)
def has_gpus():
    if os.environ.get('ODM_NO_GPU'):
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
