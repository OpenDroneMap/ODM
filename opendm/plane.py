import numpy as np
import random
import math
import os

# https://stackoverflow.com/questions/38754668/plane-fitting-in-a-3d-point-cloud

def PCA(data, correlation = False, sort = True):
    """ Applies Principal Component Analysis to the data

    Parameters
    ----------        
    data: array
        The array containing the data. The array must have NxM dimensions, where each
        of the N rows represents a different individual record and each of the M columns
        represents a different variable recorded for that individual record.
            array([
            [V11, ... , V1m],
            ...,
            [Vn1, ... , Vnm]])

    correlation(Optional) : bool
            Set the type of matrix to be computed (see Notes):
                If True compute the correlation matrix.
                If False(Default) compute the covariance matrix. 
                
    sort(Optional) : bool
            Set the order that the eigenvalues/vectors will have
                If True(Default) they will be sorted (from higher value to less).
                If False they won't.   
    Returns
    -------
    eigenvalues: (1,M) array
        The eigenvalues of the corresponding matrix.
        
    eigenvector: (M,M) array
        The eigenvectors of the corresponding matrix.

    Notes
    -----
    The correlation matrix is a better choice when there are different magnitudes
    representing the M variables. Use covariance matrix in other cases.

    """

    mean = np.mean(data, axis=0)

    data_adjust = data - mean

    #: the data is transposed due to np.cov/corrcoef syntax
    if correlation:
        
        matrix = np.corrcoef(data_adjust.T)
        
    else:
        matrix = np.cov(data_adjust.T) 

    eigenvalues, eigenvectors = np.linalg.eig(matrix)

    if sort:
        #: sort eigenvalues and eigenvectors
        sort = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[sort]
        eigenvectors = eigenvectors[:,sort]

    return eigenvalues, eigenvectors

def best_fitting_plane(points, equation=False):
    """ Computes the best fitting plane of the given points

    Parameters
    ----------        
    points: array
        The x,y,z coordinates corresponding to the points from which we want
        to define the best fitting plane. Expected format:
            array([
            [x1,y1,z1],
            ...,
            [xn,yn,zn]])
            
    equation(Optional) : bool
            Set the oputput plane format:
                If True return the a,b,c,d coefficients of the plane.
                If False(Default) return 1 Point and 1 Normal vector.    
    Returns
    -------
    a, b, c, d : float
        The coefficients solving the plane equation.

    or

    point, normal: array
        The plane defined by 1 Point and 1 Normal vector. With format:
        array([Px,Py,Pz]), array([Nx,Ny,Nz])
        
    """

    w, v = PCA(points)

    #: the normal of the plane is the last eigenvector
    normal = v[:,2]
    
    #: get a point from the plane
    point = np.mean(points, axis=0)


    if equation:
        a, b, c = normal
        d = -(np.dot(normal, point))
        return a, b, c, d
        
    else:
        return point, normal

def ransac_max_iterations(points, inliers, failure_probability):
    if len(inliers) >= len(points):
        return 0
    inlier_ratio = float(len(inliers)) / len(points)
    n = 3
    return math.log(failure_probability) / math.log(1.0 - inlier_ratio ** n)


def ransac_best_fitting_plane(points):
    if len(points) < 3:
        raise Exception("Cannot estimate plane with less than 3 points: %s" % str(points))

    max_iterations = 1000
    threshold = 1.2
    best_error = np.inf
    best_model = None
    best_inliers = []
    i = 0
    while i < max_iterations:
        samples = points[random.sample(range(len(points)), 3), :]
        model = np.array(best_fitting_plane(samples, equation=True))
        normal = model[0:3]
        normal_norm = np.linalg.norm(normal) + 1e-10

        s = points.shape[:-1] + (1,)
        hpts = np.hstack((points, np.ones(s)))

        errors = np.abs(model.T.dot(hpts.T)) / normal_norm
        errors[errors < threshold] = 0.0
        errors[errors >= threshold] = threshold + 0.1
        inliers = np.flatnonzero(np.fabs(errors) < threshold)
        error = np.fabs(errors).clip(0, threshold).sum()
        if len(inliers) and error < best_error:
            best_error = error
            best_model = model
            best_inliers = inliers
            max_iterations = min(
                max_iterations, ransac_max_iterations(points, best_inliers, 0.01)
            )
        i += 1
    
    return best_fitting_plane(points[best_inliers])


def get_z_from_XY_plane(x, y, minz, maxz, plane_normal, plane_center):
    minz -= 1e-6
    maxz += 1e-6

    b = minz - maxz

    d = minz * plane_normal[2] - maxz * plane_normal[2]
    if d == 0:
        return 0
    
    o = np.array([x,y,maxz])
    t = (plane_center.dot(plane_normal) - plane_normal.dot(o)) / d
    
    return maxz + b * t

def write_plane_ply(shots, plane_point, plane_normal, output_dir):
    # Find reconstruction X/Y boundaries on the plane based on camera shots
    minx, miny, minz, maxx, maxy, maxz = np.inf, np.inf, np.inf, -np.inf, -np.inf, -np.inf 

    for shot in shots:
        print(dir(shot))
        coords = shot.coordinates
        if coords[0] < minx:
            minx = coords[0]
        if coords[0] > maxx:
            maxx = coords[0]
        if coords[1] < miny:
            miny = coords[1]
        if coords[1] > maxy:
            maxy = coords[1]
        if coords[2] < minz:
            minz = coords[2]
        if coords[2] > maxx:
            maxx = coords[2]
    
    # Create 4 corners points
    points = np.array([
        [x, y, get_z_from_XY_plane(x, y, minz, maxz, plane_normal, plane_point)] for x,y in [[minx, miny], [minx, maxy], [maxx, miny], [maxx, maxy]]
    ])
    print(points)
    with open(os.path.join(output_dir, "plane.ply"), "wb") as fout:
        fout.write("ply\n".encode())
        fout.write("format binary_little_endian 1.0\n".encode())
        fout.write("element vertex 202141\n".encode())
        fout.write("property float32 x\n".encode())
        fout.write("property float32 y\n".encode())
        fout.write("property float32 z\n".encode())
        fout.write("element face 397578\n".encode())
        fout.write("property list uint8 uint32 vertex_indices\n".encode())
        fout.write("end_header\n".encode())