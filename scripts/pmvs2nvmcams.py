import os
import numpy as np

from opendm import log

# Go from QR-factorizatoin to corresponding RQ-factorization.
def rq(A):
    Q,R = np.linalg.qr(np.flipud(A).T)
    R = np.flipud(R.T)
    Q = Q.T
    return R[:,::-1],Q[::-1,:]

# Create a unit quaternion from rotation matrix.
def rot2quat(R):
    
    # Float epsilon (use square root to be well with the stable region).
    eps = np.sqrt(np.finfo(float).eps)
    
    # If the determinant is not 1, it's not a rotation matrix
    if np.abs(np.linalg.det(R) - 1.0) > eps:
        log.ODM_ERROR('Matrix passed to rot2quat was not a rotation matrix, det != 1.0')

    tr = np.trace(R)

    quat = np.zeros((1,4))

    # Is trace big enough be computationally stable?
    if tr > eps:
        S = 0.5 / np.sqrt(tr + 1.0)
        quat[0,0] = 0.25 / S
        quat[0,1] = (R[2,1] - R[1,2]) * S
        quat[0,2] = (R[0,2] - R[2,0]) * S
        quat[0,3] = (R[1,0] - R[0,1]) * S
    else: # It's not, use the largest diagonal.
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            quat[0,0] = (R[2,1] - R[1,2]) / S
            quat[0,1] = 0.25 * S
            quat[0,2] = (R[0,1] + R[1,0]) / S
            quat[0,3] = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 - R[0,0] + R[1,1] - R[2,2]) * 2.0
            quat[0,0] = (R[0,2] - R[2,0]) / S
            quat[0,1] = (R[0,1] + R[1,0]) / S
            quat[0,2] = 0.25 * S
            quat[0,3] = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 - R[0,0] - R[1,1] + R[2,2]) * 2.0
            quat[0,0] = (R[1,0] - R[0,1]) / S
            quat[0,1] = (R[0,2] + R[2,0]) / S
            quat[0,2] = (R[1,2] + R[2,1]) / S
            quat[0,3] = 0.25 * S

    return quat
 
# Decompose a projection matrix into parts 
# (Intrinsic projection, Rotation, Camera position)
def decomposeProjection(projectionMatrix):

    # Check input:
    if projectionMatrix.shape != (3,4):
        log.ODM_ERROR('Unable to decompose projection matrix, shape != (3,4)')

    RQ = rq(projectionMatrix[:,:3])
    
    # Fix sign, since we know K is upper triangular and has a positive diagonal.
    signMat = np.diag(np.diag(np.sign(RQ[0])))
    K = signMat*RQ[0]
    R = signMat*RQ[1]
    
    # Calculate camera position from translation vector.
    t = np.linalg.inv(-1.0*projectionMatrix[:,:3])*projectionMatrix[:,3]

    return K, R, t
   
# Parses pvms contour file.  
def parseContourFile(filePath):
   
    with open(filePath, 'r') as contourFile:
        if (contourFile.readline().strip() != "CONTOUR"):
            return np.array([])
        else:
            pMatData = np.loadtxt(contourFile, float, '#', None, None, 0)
            if pMatData.shape == (3,4):
                return pMatData
    return np.array([])


    
# Creates a .nvm camera file in the pmvs folder.
def run(pmvsFolder, outputFile):

    projectionFolder = pmvsFolder + "/txt"
    imageFolder = pmvsFolder + "/visualize"

    pMatrices = []
    imageFileNames = []
    
    # for all files in the visualize folder:
    for imageFileName in os.listdir(imageFolder):
        fileNameNoExt = os.path.splitext(imageFileName)[0]

        # look for corresponding projection matrix txt file
        projectionFilePath = os.path.join(projectionFolder, fileNameNoExt)
        projectionFilePath += ".txt"
        if os.path.isfile(projectionFilePath):
            pMatData = parseContourFile(projectionFilePath)
            if pMatData.size == 0:
                log.ODM_WARNING('Unable to parse contour file, skipping: %s'
                                % projectionFilePath)
            else:
                pMatrices.append(np.matrix(pMatData))
                imageFileNames.append(imageFileName)

    
    # Decompose projection matrices
    focals = []
    rotations = []
    translations = []
    for projection in pMatrices:
        KRt = decomposeProjection(projection)
        focals.append(KRt[0][0,0])
        rotations.append(rot2quat(KRt[1]))
        translations.append(KRt[2])

    # Create .nvm file
    with open (outputFile, 'w') as nvmFile:
        nvmFile.write("NVM_V3\n\n")
        nvmFile.write('%d' % len(rotations) + "\n")
        
        for idx, imageFileName in enumerate(imageFileNames):
            nvmFile.write(os.path.join("visualize", imageFileName))
            nvmFile.write(" " + '%f' % focals[idx])
            nvmFile.write(" " + '%f' % rotations[idx][0,0] +
                          " " + '%f' % rotations[idx][0,1] +
                          " " + '%f' % rotations[idx][0,2] +
                          " " + '%f' % rotations[idx][0,3])
            nvmFile.write(" " + '%f' % translations[idx][0] +
                          " " + '%f' % translations[idx][1] +
                          " " + '%f' % translations[idx][2])
            nvmFile.write(" 0 0\n")
        nvmFile.write("0\n\n")
        nvmFile.write("0\n\n")
        nvmFile.write("0")
