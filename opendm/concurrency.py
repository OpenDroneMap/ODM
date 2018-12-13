from psutil import virtual_memory
import os

def get_max_memory(minimum = 5, use_at_most = 0.5):
    """
    :param minimum minimum value to return (return value will never be lower than this)
    :param use_at_most use at most this fraction of the available memory. 0.5 = use at most 50% of available memory
    :return percentage value of memory to use (75 = 75%).
    """
    return max(minimum, (100 - virtual_memory().percent) * use_at_most)


def get_max_concurrency_for_dem(available_cores, input_file, use_at_most = 0.8):
    """
    DEM generation requires ~2x the input file size of memory per available core.
    :param available_cores number of cores available (return value will never exceed this value)
    :param input_file path to input file
    :use_at_most use at most this fraction of the available memory when calculating a concurrency value. 0.9 = assume that we can only use 90% of available memory.
    :return maximum number of cores recommended to use for DEM processing.
    """
    memory_available = virtual_memory().available * use_at_most
    file_size = os.path.getsize(input_file)
    memory_required_per_core = max(1, file_size * 2)
    
    return min(available_cores, max(1, int(memory_available) / int(memory_required_per_core)))

