#Created on 5/28/2022 by Josh von Nonn.
#Built from source code provided by Brad Chmambers - based off Mungus et al.,2014. (Apache 2.0 License)


#This algorithm is currently optimized for the Mismibazi dataset with a 1 meter resolution. Future use with this tool
# is expected for user to enter a categorical value contingent upon topographical complexity. These categories will be 
# pre-optimized.


from scipy import ndimage, signal, spatial
from scipy.ndimage import morphology
import numpy as np
import pandas as pd
import pdal



def idw(data):
    # Find indices of the ground returns, i.e., anything that is not a nan, and create a KD-tree.
    # We will search this tree when looking for nearest neighbors to perform the interpolation.
    valid = np.argwhere(~np.isnan(data))
    tree = spatial.cKDTree(valid)
    
    # Now find indices of the non-ground returns, as indicated by nan values. We will interpolate
    # at these locations.
    nans = np.argwhere(np.isnan(data))    
    for row in nans:
        d, idx = tree.query(row, k=12) #k = number of nearest neighbors
        d = np.power(d, -2) #each item in d raised to its reciprocated power (basis of idw) the value "r" also defines the smoothness of the interpolation
        v = data[valid[idx, 0], valid[idx, 1]] 
        data[row[0], row[1]] = np.inner(v, d)/np.sum(d) #nans are replaced with the result of (v * d)/sum(d)
        
    return data


def dmpdtm(fin, fout):

    #readin in full res .las , subsampled with Poisson- change radius to reach desired resolution
    p = pdal.Reader.las(fin).pipeline() | pdal.Filter.sample(radius=1).pipeline()
    p.execute()

    cls = p.arrays[0]['Classification']
    cls.fill(1)

    df3D = pd.DataFrame(p.arrays[0], columns=['X','Y','Z'])

    #define variables (if we keep k = 0, then I'll clean up the code, remove gstar?)
    S = 10
    k = 0.000
    n = 0.1
    b = -0.2

    hres = 1

    #np.ogrid "open-grid", creates a way to index the matrix (access pixels/pts) hres is the step
    xi = np.ogrid[p.arrays[0]['X'].min():p.arrays[0]['X'].max():hres]
    yi = np.ogrid[p.arrays[0]['Y'].min():p.arrays[0]['Y'].max():hres]

    #np.digitize allocates points to bins and then bins are grouped in the df
    bins = df3D.groupby([np.digitize(p.arrays[0]['X'], xi), np.digitize(p.arrays[0]['Y'], yi)])

    zmins = bins.Z.min() #collects the lowest point in each bin
    cz = np.empty((yi.size, xi.size)) 
    cz.fill(np.nan) 
    for name, val in zmins.iteritems():
        cz[name[1]-1, name[0]-1] = val #adding coordinates to lowest points only

    cz = idw(cz)

    #23x,23y structuring element for opening
    struct = ndimage.iterate_structure(ndimage.generate_binary_structure(2, 1), 11).astype(int)
    opened = morphology.grey_opening(cz, structure=struct) 

    #19x,19y structuring element for closing
    struct = ndimage.iterate_structure(ndimage.generate_binary_structure(2, 1), 9).astype(int)
    closed = morphology.grey_closing(opened, structure=struct)

    #removing low outliers: if any pt in cz is >= 1 meter below the surface of closed then it is set to the 
    #closed surface value
    #need to test lower limit >= 0.5
    lowx, lowy = np.where((closed - cz) >= 1.0) 
    cz[lowx, lowy] = closed[lowx, lowy]

    stdev = 14
    #product of two guassian arrays with the max normalized to 1, size/window = 113
    G = np.outer(signal.gaussian(113,stdev), signal.gaussian(113,stdev))
    #fast fourier transform convolution, matrix is padded at 2*stdev
    low = signal.fftconvolve(np.pad(cz,2*stdev,'edge'), G, mode='same')[2*stdev:-2*stdev,2*stdev:-2*stdev]/1000.

    high = cz - low

    erosions = []
    granulometry = []
    erosions.append(morphology.grey_erosion(high, size=3))
    for scale in range(1, S):
        erosions.append(morphology.grey_erosion(erosions[scale-1], size=3))
    for scale in range(1, S+1):
        granulometry.append(morphology.grey_dilation(erosions[scale-1], size=2*scale+1))

    out = []
    for i in range(1, len(granulometry)):
        out.append(granulometry[i-1]-granulometry[i])

    gprime = np.maximum.reduce(out)
    #gstar and gplus may be necessary for more alt. terrain optimization

    #xs, ys = out[0].shape
    #gstar = np.zeros((xs,ys))
    #gplus = np.zeros((xs,ys))
    #for ii in range(0,xs):
    #    for jj in range(0,ys):
    #        for kk in range(0,len(out)):
    #            if out[kk][ii,jj] < gprime[ii,jj]:
    #                gplus[ii,jj] += out[kk][ii,jj]
    #            if out[kk][ii,jj] == gprime[ii,jj]:
    #               gplus[ii,jj] += out[kk][ii,jj]
    #                #gstar[ii,jj] = kk
    #                break

    #T = k * gstar + n
    Sg = gprime < n

    F = cz.copy()
    F[np.where(Sg==0)] = np.nan

    G = idw(F)

    struct = ndimage.iterate_structure(ndimage.generate_binary_structure(2, 1), 1).astype(int)
    gradDTM = morphology.grey_dilation(G, structure=struct)

    xbins = np.digitize(df3D.X, xi)
    ybins = np.digitize(df3D.Y, yi)

    #nonground = np.where(df3D.Z >= gradDTM[ybins-1, xbins-1]+b)
    ground = np.where(df3D.Z < gradDTM[ybins-1, xbins-1]+b)

    cls[ground] = 2 #set ground points to 2

    output = p.arrays[0]
    output['Classification'] =cls

    p = pdal.Filter.range(limits="Classification[2:2]").pipeline(output)
    p.execute()

    #default Progressive morphological filter stacked to catch stragglers (havent'tested with alt parameters)
    #need to test with alt smrf
    pmf_arr = p.arrays[0]
    p = pdal.Filter.pmf().pipeline(pmf_arr) | pdal.Filter.range(limits="Classification[2:2]").pipeline()
    p.execute()

    
    #write out las file with ground points only 
#    outputfile = lasfile.replace(".las","_dtm.las")
    final_out = p.arrays[0]
    p = pdal.Writer.las(filename= fout).pipeline(final_out)
    p.execute()

    return outputfile
