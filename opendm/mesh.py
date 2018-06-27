import os, shutil, sys, struct
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
    radius_steps = [dsm_resolution, dsm_resolution * 3, dsm_resolution * 9]

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
    if os.path.exists(tmp_directory):
        shutil.rmtree(tmp_directory)

    return mesh

def dem_to_points(inGeotiff, outPointCloud):
    log.ODM_INFO('Sampling points from DSM: %s' % inGeotiff)

    image = GeoImage.open([inGeotiff], bandnames=['z'], nodata=-9999)
    arr = image['z'].read_raw()

    # Median filter
    log.ODM_INFO('Applying median filter...')

    arr = signal.medfilt(arr, 1)

    log.ODM_INFO('Writing points...')

    with open(outPointCloud, "wb") as f:
        f.write("ply\n")
        f.write("format binary_%s_endian 1.0\n" % sys.byteorder) 
        f.write("element vertex %s\n" % arr.size)
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property float nx\n")
        f.write("property float ny\n")
        f.write("property float nz\n")
        f.write("end_header\n")

        xmin, xmax, ymin, ymax = image.extent().x0(), image.extent().x1(), image.extent().y0(), image.extent().y1()
        ext_width, ext_height = xmax - xmin, ymax - ymin
        arr_height, arr_width = arr.shape

        for x in range(arr_width):
            for y in range(arr_height):
                tx = xmin + (float(x) / float(arr_width)) * ext_width
                ty = ymax - (float(y) / float(arr_height)) * ext_height
                f.write(struct.pack('ffffff', tx, ty, arr[y][x], 0, 0, 1))
                #f.write('%s %s %s\n' % (tx, ty, z))

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
             '--normals --linearFit '
             '{verbose}'.format(**kwargs))

    return outMesh
