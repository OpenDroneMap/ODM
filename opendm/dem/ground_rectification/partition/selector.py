from .one_partition import OnePartition
from .quad_partitions import UniformPartitions, MedianPartitions
from .surrounding_partitions import SurroundingPartitions


def select_partition_plan(name, point_cloud):
    if name == 'one':
        return OnePartition(point_cloud)
    elif name == 'uniform':
        return UniformPartitions(point_cloud)
    elif name == 'median':
        return MedianPartitions(point_cloud)
    elif name == 'surrounding':
        return SurroundingPartitions(point_cloud)
    else:
        raise Exception('Incorrect partition name.')
