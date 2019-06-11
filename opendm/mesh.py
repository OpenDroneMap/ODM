from __future__ import absolute_import
import os, shutil, sys, struct, random, math
from opendm.dem import commands
from opendm import system
from opendm import log
from opendm import context
from scipy import signal, ndimage
import numpy as np

def create_25dmesh(inPointCloud, outMesh, dsm_radius=0.07, dsm_resolution=0.05, depth=8, samples=1, maxVertexCount=100000, verbose=False, available_cores=None, method='gridded', smooth_dsm=True):
    # Create DSM from point cloud

    # Create temporary directory
    mesh_directory = os.path.dirname(outMesh)
    tmp_directory = os.path.join(mesh_directory, 'tmp')
    if os.path.exists(tmp_directory):
        shutil.rmtree(tmp_directory)
    os.mkdir(tmp_directory)
    log.ODM_INFO('Created temporary directory: %s' % tmp_directory)

    radius_steps = [dsm_radius]

    log.ODM_INFO('Creating DSM for 2.5D mesh')

    commands.create_dem(
            inPointCloud,
            'mesh_dsm',
            output_type='max',
            radiuses=map(str, radius_steps),
            gapfill=True,
            outdir=tmp_directory,
            resolution=dsm_resolution,
            verbose=verbose,
            max_workers=available_cores,
            apply_smoothing=smooth_dsm
        )

    if method == 'gridded':
        mesh = dem_to_mesh_gridded(os.path.join(tmp_directory, 'mesh_dsm.tif'), outMesh, maxVertexCount, verbose)
    elif method == 'poisson':
        dsm_points = dem_to_points(os.path.join(tmp_directory, 'mesh_dsm.tif'), os.path.join(tmp_directory, 'dsm_points.ply'), verbose)
        mesh = screened_poisson_reconstruction(dsm_points, outMesh, depth=depth, 
                                    samples=samples, 
                                    maxVertexCount=maxVertexCount, 
                                    threads=max(1, available_cores - 1), # poissonrecon can get stuck on some machines if --threads == all cores
                                    verbose=verbose)
    else:
        raise 'Not a valid method: ' + method

    # Cleanup tmp
    if os.path.exists(tmp_directory):
        shutil.rmtree(tmp_directory)

    return mesh


def dem_to_points(inGeotiff, outPointCloud, verbose=False):
    log.ODM_INFO('Sampling points from DSM: %s' % inGeotiff)

    kwargs = {
        'bin': context.dem2points_path,
        'outfile': outPointCloud,
        'infile': inGeotiff,
        'verbose': '-verbose' if verbose else ''
    }

    system.run('{bin} -inputFile {infile} '
         '-outputFile {outfile} '
         '-skirtHeightThreshold 1.5 '
         '-skirtIncrements 0.2 '
         '-skirtHeightCap 100 '
         ' {verbose} '.format(**kwargs))

    return outPointCloud


def dem_to_mesh_gridded(inGeotiff, outMesh, maxVertexCount, verbose=False):
    log.ODM_INFO('Creating mesh from DSM: %s' % inGeotiff)

    mesh_path, mesh_filename = os.path.split(outMesh)
    # mesh_path = path/to
    # mesh_filename = odm_mesh.ply

    basename, ext = os.path.splitext(mesh_filename)
    # basename = odm_mesh
    # ext = .ply

    outMeshDirty = os.path.join(mesh_path, "{}.dirty{}".format(basename, ext))

    kwargs = {
        'bin': context.dem2mesh_path,
        'outfile': outMeshDirty,
        'infile': inGeotiff,
        'maxVertexCount': maxVertexCount,
        'verbose': '-verbose' if verbose else ''
    }

    system.run('{bin} -inputFile {infile} '
         '-outputFile {outfile} '
         '-maxTileLength 4000 '
         '-maxVertexCount {maxVertexCount} '
         ' {verbose} '.format(**kwargs))

    # Cleanup and reduce vertex count if necessary 
    # (as dem2mesh cannot guarantee that we'll have the target vertex count)
    cleanupArgs = {
        'bin': context.odm_modules_path,
        'outfile': outMesh,
        'infile': outMeshDirty,
        'max_vertex': maxVertexCount,
        'verbose': '-verbose' if verbose else ''
    }

    system.run('{bin}/odm_cleanmesh -inputFile {infile} '
         '-outputFile {outfile} '
         '-removeIslands '
         '-decimateMesh {max_vertex} {verbose} '.format(**cleanupArgs))

    # Delete intermediate results
    os.remove(outMeshDirty)

    return outMesh


def screened_poisson_reconstruction(inPointCloud, outMesh, depth = 8, samples = 1, maxVertexCount=100000, pointWeight=4, threads=context.num_cores, verbose=False):

    mesh_path, mesh_filename = os.path.split(outMesh)
    # mesh_path = path/to
    # mesh_filename = odm_mesh.ply

    basename, ext = os.path.splitext(mesh_filename)
    # basename = odm_mesh
    # ext = .ply

    outMeshDirty = os.path.join(mesh_path, "{}.dirty{}".format(basename, ext))

    poissonReconArgs = {
      'bin': context.poisson_recon_path,
      'outfile': outMeshDirty,
      'infile': inPointCloud,
      'depth': depth,
      'samples': samples,
      'pointWeight': pointWeight,
      'threads': threads,
      'verbose': '--verbose' if verbose else ''
    }

    # Run PoissonRecon
    system.run('{bin} --in {infile} '
             '--out {outfile} '
             '--depth {depth} '
             '--pointWeight {pointWeight} '
             '--samplesPerNode {samples} '
             '--threads {threads} '
             '--linearFit '
             '{verbose}'.format(**poissonReconArgs))

    # Cleanup and reduce vertex count if necessary
    cleanupArgs = {
        'bin': context.odm_modules_path,
        'outfile': outMesh,
        'infile': outMeshDirty,
        'max_vertex': maxVertexCount,
        'verbose': '-verbose' if verbose else ''
    }

    system.run('{bin}/odm_cleanmesh -inputFile {infile} '
         '-outputFile {outfile} '
         '-removeIslands '
         '-decimateMesh {max_vertex} {verbose} '.format(**cleanupArgs))

    # Delete intermediate results
    os.remove(outMeshDirty)

    return outMesh
