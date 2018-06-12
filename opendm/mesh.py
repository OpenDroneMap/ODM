from __future__ import absolute_import
import os, shutil, sys, struct
from io import BytesIO
from gippy import GeoImage
from opendm.dem import commands
from opendm import system
from opendm import log
from scipy import signal

def create_25dmesh(inPointCloud, outMesh, dsm_resolution=0.05, depth=8, samples=1, verbose=False):
    # Create DSM from point cloud

    # Create temporary directory
    mesh_directory = os.path.dirname(outMesh)
    tmp_directory = os.path.join(mesh_directory, 'tmp')
    if os.path.exists(tmp_directory):
        shutil.rmtree(tmp_directory)
    os.mkdir(tmp_directory)
    log.ODM_INFO('Created temporary directory: %s' % tmp_directory)

    # Always use just two steps
    radius_steps = [dsm_resolution / 8.0, dsm_resolution / 4.0, dsm_resolution / 2.0, dsm_resolution]

    log.ODM_INFO('Creating DSM for 2.5D mesh')

    commands.create_dems(
            [inPointCloud],
            'mesh_dsm',
            radius=map(str, radius_steps),
            gapfill=True,
            outdir=tmp_directory,
            resolution=dsm_resolution,
            verbose=verbose
        )

    dsm_points = dem_to_points(os.path.join(tmp_directory, 'mesh_dsm.tif'), os.path.join(tmp_directory, 'dsm_points.ply'))
    mesh = screened_poisson_reconstruction(dsm_points, outMesh, depth=depth, samples=samples, verbose=verbose)
    
    # Cleanup tmp
    #if os.path.exists(tmp_directory):
    #    shutil.rmtree(tmp_directory)

    return mesh

def dem_to_points(inGeotiff, outPointCloud):
    log.ODM_INFO('Sampling points from DSM: %s' % inGeotiff)

    image = GeoImage.open([inGeotiff], bandnames=['z'], nodata=-9999)
    arr = image['z'].read_raw()
    resolution = max(abs(image.resolution().x()), abs(image.resolution().y()))

    # Median filter
    log.ODM_INFO('Applying median filter...')

    arr = signal.medfilt2d(arr, 5)

    log.ODM_INFO('Writing points...')

    mem_file = BytesIO()

    xmin, xmax, ymin, ymax = image.extent().x0(), image.extent().x1(), image.extent().y0(), image.extent().y1()
    ext_width, ext_height = xmax - xmin, ymax - ymin
    arr_height, arr_width = arr.shape
    vertex_count = (arr_height - 4) * (arr_width - 4)
    skirt_points = 0

    skirt_height_threshold = 1 # meter
    skirt_increments = resolution

    for x in range(2, arr_width - 2):
        for y in range(2, arr_height - 2):
            z = arr[y][x]
            tx = xmin + (float(x) / float(arr_width)) * ext_width
            ty = ymax - (float(y) / float(arr_height)) * ext_height
            mem_file.write(struct.pack('ffffff', tx, ty, z, 0, 0, 1))
            
            # Skirting
            for (nx, ny) in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                current_z = z
                neighbor_z = arr[nx][ny]
                if current_z - neighbor_z > skirt_height_threshold:
                    stop_at_z = neighbor_z + skirt_increments
                    while current_z > stop_at_z:
                        current_z -= skirt_increments
                        mem_file.write(struct.pack('ffffff', tx, ty, current_z, 0, 0, 1))
                        skirt_points += 1

    with open(outPointCloud, "wb") as f:
        f.write("ply\n")
        f.write("format binary_%s_endian 1.0\n" % sys.byteorder) 
        f.write("element vertex %s\n" % (vertex_count + skirt_points))
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property float nx\n")
        f.write("property float ny\n")
        f.write("property float nz\n")
        f.write("end_header\n")
        f.write(mem_file.getvalue())

    mem_file.close()

    log.ODM_INFO("Points count: %s (%s samples, %s skirts)", vertex_count + skirt_points, vertex_count, skirt_points)
    log.ODM_INFO('Wrote points to: %s' % outPointCloud)

    return outPointCloud


def screened_poisson_reconstruction(inPointCloud, outMesh, depth = 8, samples = 1, verbose=False):
    #TODO: @dakotabenjamin adjust path to PoissonRecon program
    kwargs = {
      'bin': '/PoissonRecon/Bin/Linux/PoissonRecon',
      'outfile': outMesh,
      'infile': inPointCloud,
      'depth': depth,
      'samples': samples,
      'verbose': '--verbose' if verbose else ''
    }

    # Run PoissonRecon
    system.run('{bin} --in {infile} '
             '--out {outfile} '
             '--depth {depth} '
             '--linearFit '
             '{verbose}'.format(**kwargs))

    return outMesh
