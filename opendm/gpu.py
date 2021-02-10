from opendm import log
from repoze.lru import lru_cache

@lru_cache(maxsize=None)
def has_gpus():
    import pyopencl

    try:
        platforms = pyopencl.get_platforms()
        for p in platforms:
            log.ODM_INFO("Found GPU device: %s" % p.name)

        return len(platforms) > 0
    except Exception as e:
        return False
