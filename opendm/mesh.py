from __future__ import absolute_import
import os, shutil, sys, struct, random, math, platform
from opendm.dem import commands
from opendm import system
from opendm import log
from opendm import context
from opendm import concurrency
from opendm import point_cloud
from scipy import signal
import numpy as np

def create_25dmesh(inPointCloud, outMesh, radius_steps=["0.05"], dsm_resolution=0.05, depth=8, samples=1, maxVertexCount=100000, available_cores=None, method='gridded', smooth_dsm=True, max_tiles=None):
    # Create DSM from point cloud

    # Create temporary directory
    mesh_directory = os.path.dirname(outMesh)
    tmp_directory = os.path.join(mesh_directory, 'tmp')
    if os.path.exists(tmp_directory):
        shutil.rmtree(tmp_directory)
    os.mkdir(tmp_directory)
    log.ODM_INFO('Created temporary directory: %s' % tmp_directory)

    log.ODM_INFO('Creating DSM for 2.5D mesh')

    commands.create_dem(
            inPointCloud,
            'mesh_dsm',
            output_type='max',
            radiuses=radius_steps,
            gapfill=True,
            outdir=tmp_directory,
            resolution=dsm_resolution,
            max_workers=available_cores,
            apply_smoothing=smooth_dsm,
            max_tiles=max_tiles
        )

    if method == 'gridded':
        mesh = dem_to_mesh_gridded(os.path.join(tmp_directory, 'mesh_dsm.tif'), outMesh, maxVertexCount, maxConcurrency=max(1, available_cores))
    elif method == 'poisson':
        dsm_points = dem_to_points(os.path.join(tmp_directory, 'mesh_dsm.tif'), os.path.join(tmp_directory, 'dsm_points.ply'))
        mesh = screened_poisson_reconstruction(dsm_points, outMesh, depth=depth, 
                                    samples=samples, 
                                    maxVertexCount=maxVertexCount, 
                                    threads=max(1, available_cores - 1)), # poissonrecon can get stuck on some machines if --threads == all cores
    else:
        raise 'Not a valid method: ' + method

    # Cleanup tmp
    if os.path.exists(tmp_directory):
        shutil.rmtree(tmp_directory)

    return mesh


def dem_to_points(inGeotiff, outPointCloud):
    log.ODM_INFO('Sampling points from DSM: %s' % inGeotiff)

    kwargs = {
        'bin': context.dem2points_path,
        'outfile': outPointCloud,
        'infile': inGeotiff
    }

    system.run('"{bin}" -inputFile "{infile}" '
         '-outputFile "{outfile}" '
         '-skirtHeightThreshold 1.5 '
         '-skirtIncrements 0.2 '
         '-skirtHeightCap 100 '
         '-verbose '.format(**kwargs))

    return outPointCloud


def dem_to_mesh_gridded(inGeotiff, outMesh, maxVertexCount, maxConcurrency=1):
    log.ODM_INFO('Creating mesh from DSM: %s' % inGeotiff)

    mesh_path, mesh_filename = os.path.split(outMesh)
    # mesh_path = path/to
    # mesh_filename = odm_mesh.ply

    basename, ext = os.path.splitext(mesh_filename)
    # basename = odm_mesh
    # ext = .ply

    outMeshDirty = os.path.join(mesh_path, "{}.dirty{}".format(basename, ext))

    # This should work without issues most of the times, 
    # but just in case we lower maxConcurrency if it fails.
    while True:
        try:
            kwargs = {
                'bin': context.dem2mesh_path,
                'outfile': outMeshDirty,
                'infile': inGeotiff,
                'maxVertexCount': maxVertexCount,
                'maxConcurrency': maxConcurrency
            }
            system.run('"{bin}" -inputFile "{infile}" '
                '-outputFile "{outfile}" '
                '-maxTileLength 2000 '
                '-maxVertexCount {maxVertexCount} '
                '-maxConcurrency {maxConcurrency} '
                '-edgeSwapThreshold 0.15 '
                '-verbose '.format(**kwargs))
            break
        except Exception as e:
            maxConcurrency = math.floor(maxConcurrency / 2)
            if maxConcurrency >= 1:
                log.ODM_WARNING("dem2mesh failed, retrying with lower concurrency (%s) in case this is a memory issue" % maxConcurrency)
            else:
                raise e


    # Cleanup and reduce vertex count if necessary 
    # (as dem2mesh cannot guarantee that we'll have the target vertex count)
    cleanupArgs = {
        'reconstructmesh': context.omvs_reconstructmesh_path,
        'outfile': outMesh,
        'infile': outMeshDirty,
        'max_faces': maxVertexCount * 2
    }

    system.run('"{reconstructmesh}" -i "{infile}" '
         '-o "{outfile}" '
         '--archive-type 3 '
         '--remove-spikes 0 --remove-spurious 0 --smooth 0 '
         '--target-face-num {max_faces} -v 0'.format(**cleanupArgs))

    # Delete intermediate results
    os.remove(outMeshDirty)

    return outMesh


def screened_poisson_reconstruction(inPointCloud, outMesh, depth = 8, samples = 1, maxVertexCount=100000, pointWeight=4, threads=context.num_cores):

    mesh_path, mesh_filename = os.path.split(outMesh)
    # mesh_path = path/to
    # mesh_filename = odm_mesh.ply

    basename, ext = os.path.splitext(mesh_filename)
    # basename = odm_mesh
    # ext = .ply

    outMeshDirty = os.path.join(mesh_path, "{}.dirty{}".format(basename, ext))
    if os.path.isfile(outMeshDirty):
        os.remove(outMeshDirty)
    
    # Since PoissonRecon has some kind of a race condition on ppc64el, and this helps...
    if platform.machine() == 'ppc64le':
        log.ODM_WARNING("ppc64le platform detected, forcing single-threaded operation for PoissonRecon")
        threads = 1

    while True:
        poissonReconArgs = {
            'bin': context.poisson_recon_path,
            'outfile': outMeshDirty,
            'infile': inPointCloud,
            'depth': depth,
            'samples': samples,
            'pointWeight': pointWeight,
            'threads': int(threads)
        }

        # Run PoissonRecon
        try:
            system.run('"{bin}" --in "{infile}" '
                    '--out "{outfile}" '
                    '--depth {depth} '
                    '--pointWeight {pointWeight} '
                    '--samplesPerNode {samples} '
                    '--threads {threads} '
                    '--bType 2 '
                    '--linearFit '.format(**poissonReconArgs))
        except Exception as e:
            log.ODM_WARNING(str(e))
            
        if os.path.isfile(outMeshDirty):
            break # Done!
        else:

            # PoissonRecon will sometimes fail due to race conditions
            # on certain machines, especially on Windows
            threads //= 2

            if threads < 1:
                break
            else:
                log.ODM_WARNING("PoissonRecon failed with %s threads, let's retry with %s..." % (threads * 2, threads))


    # Cleanup and reduce vertex count if necessary
    cleanupArgs = {
        'reconstructmesh': context.omvs_reconstructmesh_path,
        'outfile': outMesh,
        'infile':outMeshDirty,
        'max_faces': maxVertexCount * 2
    }

    system.run('"{reconstructmesh}" -i "{infile}" '
         '-o "{outfile}" '
         '--archive-type 3 '
         '--remove-spikes 0 --remove-spurious 20 --smooth 0 '
         '--target-face-num {max_faces} -v 0'.format(**cleanupArgs))

    # Delete intermediate results
    os.remove(outMeshDirty)

    return outMesh
