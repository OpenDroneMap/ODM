import os, subprocess
from opendm import log
from repoze.lru import lru_cache

@lru_cache(maxsize=None)
def has_gpus():
    import pyopencl

    try:
        platforms = pyopencl.get_platforms()
        print(platforms)
        exit(1)
    except Exception as e:
        print(e)
        return False
