from psutil import virtual_memory
import os

def get_max_memory(minimum = 5, use_at_most = 0.5):
    """
    :param minimum minimum value to return (return value will never be lower than this)
    :param use_at_most use at most this fraction of the available memory. 0.5 = use at most 50% of available memory
    :return percentage value of memory to use (75 = 75%).
    """
    return max(minimum, (100 - virtual_memory().percent) * use_at_most)

def get_max_memory_mb(minimum = 100, use_at_most = 0.5):
    """
    :param minimum minimum value to return (return value will never be lower than this)
    :param use_at_most use at most this fraction of the available memory. 0.5 = use at most 50% of available memory
    :return value of memory to use in megabytes.
    """
    return max(minimum, (virtual_memory().available / 1024 / 1024) * use_at_most)
