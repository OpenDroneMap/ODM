import os
import math
import subprocess

# Purpose:
# Preselect camera pairs for meature matching based by using GPS exif data.

def preselect_pairs(utm_extractor_path, image_list, k_distance, use_knn_mode = True, verbose = False):

    # Parameterss:
    # utm_extractor_path: Path to the odm utm_extraction program that parses exif data.
    # images_path:        Path to the folder containing the images.
    # k_distance:         Nearest neighbor count in case of use_knn_mode = True, otherwise
    #                     the minimum distance between cameras to be matched.
    # use_knn_mode:       True if knn mode is used to preselect matches, False for distance thresholding.     

# Temporary files
    image_list_file = 'image_list_tmp.txt'
    utm_coord_file = 'utm_coord_tmp.txt'

# Parse the list of files with successfullt detectd points.
    image_folder = ""
    filenames = []
    with open(image_list, 'r') as keypoint_file:
        first_line = keypoint_file.readline()
        image_folder = first_line[:first_line.rfind("/")]
        keypoint_file.seek(0)
        for line in keypoint_file:
            filename = line[line.rfind("/")+1:line.rfind(".")]
            filename += ".jpg"
            filenames.append(filename)
    
    data = open(image_list_file, "w")
    for filename in filenames:
        data.write("%s\n" % filename)
    data.close()


# Define call parameters for calling the odm_extract_utm module
    utm_cmd = ''
    if verbose:
        utm_cmd = [utm_extractor_path, '-verbose', '-imagesPath', image_folder, \
                   '-imageListFile', image_list_file, '-outputCoordFile', utm_coord_file]
    else:
        utm_cmd = [utm_extractor_path, '-imagesPath', image_folder, '-imageListFile', \
                   image_list_file, '-outputCoordFile', utm_coord_file]

# Perform UTM extraction
    utm_process = subprocess.Popen(utm_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output, err = utm_process.communicate()

# Check return code
    if utm_process.returncode != 0:
        print "Error when performing utm_extraction, dumping log file:"
        print output
        os.remove(image_list_file)
        return []

# Parse odm_extract_utm output
    coords = []
    with open('utm_coord_tmp.txt', 'r') as coord_file:
    # Skip header lines
        next(coord_file)
        next(coord_file)
        for line in coord_file:
            first_space = line.find(' ')
            second_space = first_space + line[first_space+1:].find(' ')
            xCoord = float(line[0: first_space])
            yCoord = float(line[first_space+1: second_space+1])
            coords.append([xCoord, yCoord])

# Calculate distances between camera pairs
    distances = []
    for xo,yo in coords:
        distances.append([])
        for xi, yi in coords: 
            current_image = len(distances)
            xDist = xo-xi
            yDist = yo-yi
            distance = math.sqrt(xDist*xDist + yDist*yDist)
            current_image = len(distances[-1])
            distances[-1].append([current_image, distance, False])
        distances[-1].sort(key=lambda tup: tup[1])

# Select matched pairs and make there are no doubles
    matched_pairs = []   
    if use_knn_mode:
        # Make sure that image count > k
        if len(coords) <= k_distance:
            print "Warning, k >= image count, the result will be equivalent to exhaustive matching"
            k_distance = len(coords)-1
        for outer_index, sorted_list in enumerate(distances):
            # Skip first element as distance will always be zero
            for sorted_index in xrange(1, k_distance+1):
                image_index, distance, dummy = distances[outer_index][sorted_index]
                is_added = distances[outer_index][image_index][2]
                if not is_added:
                    matched_pairs.append([outer_index, image_index])
                    distances[outer_index][image_index][2] = True
                    distances[image_index][outer_index][2] = True
    else: # Distance mode
        for outer_index, sorted_list in enumerate(distances):
            # Skip first element as distance will always be zero
            for image_index, distance, dummy in sorted_list:
                if outer_index == image_index:
                    continue
                if distance > k_distance:
                    break
                is_added = distances[outer_index][image_index][2]
                if not is_added:
                    matched_pairs.append([outer_index, image_index])
                    distances[outer_index][image_index][2] = True
                    distances[image_index][outer_index][2] = True

    # Remove temporary files
    os.remove(image_list_file)
    os.remove(utm_coord_file)    
    
    return matched_pairs
