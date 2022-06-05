import math
import re
import cv2
import os
from opendm import dls
import numpy as np
from opendm import log
from opendm.concurrency import parallel_map
from opensfm.io import imread

from skimage import exposure
from skimage.morphology import disk
from skimage.filters import rank, gaussian

# Loosely based on https://github.com/micasense/imageprocessing/blob/master/micasense/utils.py

# def dn_to_radiance(photo, image):                                                                                         # Cam--
def dn_to_radiance(photo, args, image):                                                                                     # Cam++
    """
    Convert Digital Number values to Radiance values
    :param photo ODM_Photo
    :param image numpy array containing image data
    :return numpy array with radiance image values
    """

    image = image.astype("float32")
    if len(image.shape) != 3:
        raise ValueError("Image should have shape length of 3 (got: %s)" % len(image.shape))

    # Handle thermal bands (experimental)
    if photo.band_name == 'LWIR':
        image -= (273.15 * 100.0) # Convert Kelvin to Celsius
        image *= 0.01
        return image
    
    if photo.camera_make == 'Parrot' and photo.camera_model == 'Sequoia':                                                   # Cam(Seq)++
        dark_level = photo.get_dark_level()                                                                                 # Cam(Seq)++
        if photo.band_name == 'Green':                                                                                      # Cam(Seq)++
            if args.radiometric_seq_darklevel_green != 0.0:                                                                 # Cam(Seq)++
                dark_level = args.radiometric_seq_darklevel_green                                                           # Cam(Seq)++
        elif photo.band_name == 'Red':                                                                                      # Cam(Seq)++
            if args.radiometric_seq_darklevel_red != 0.0:                                                                   # Cam(Seq)++
                dark_level = args.radiometric_seq_darklevel_red                                                             # Cam(Seq)++
        elif photo.band_name == 'Rededge' or photo.band_name == 'RedEdge':                                                  # Cam(Seq)++
            if args.radiometric_seq_darklevel_rededge != 0.0:                                                               # Cam(Seq)++
                dark_level = args.radiometric_seq_darklevel_rededge                                                         # Cam(Seq)++
        elif photo.band_name == 'NIR':                                                                                      # Cam(Seq)++
            if args.radiometric_seq_darklevel_nir != 0.0:                                                                   # Cam(Seq)++
                dark_level = args.radiometric_seq_darklevel_nir                                                             # Cam(Seq)++
                                                                                                                            # Cam(Seq)++
        exposure_time = photo.exposure_time                                                                                 # Cam(Seq)++
        if exposure_time is not None:                                                                                       # Cam(Seq)++
            if photo.band_name == 'Green':                                                                                  # Cam(Seq)++
                exposure_time += args.radiometric_seq_exposuretimezero_green                                                # Cam(Seq)++
            elif photo.band_name == 'Red':                                                                                  # Cam(Seq)++
                exposure_time += args.radiometric_seq_exposuretimezero_red                                                  # Cam(Seq)++
            elif photo.band_name == 'Rededge' or photo.band_name == 'RedEdge':                                              # Cam(Seq)++
                exposure_time += args.radiometric_seq_exposuretimezero_rededge                                              # Cam(Seq)++
            elif photo.band_name == 'NIR':                                                                                  # Cam(Seq)++
                exposure_time += args.radiometric_seq_exposuretimezero_nir                                                  # Cam(Seq)++
                                                                                                                            # Cam(Seq)++
        gain = photo.get_gain()                                                                                             # Cam(Seq)++
        a1, a2, a3 = photo.get_radiometric_calibration()                                                                    # Cam(Seq)++
                                                                                                                            # Cam(Seq)++
        if dark_level is None or exposure_time is None or gain is None or a1 is None:                                       # Cam(Seq)++
            log.ODM_ERROR("Error {} Invalid_EXIF_tags".format(photo.filename))                                              # Cam(Seq)++
            raise RuntimeError("Invalid_EXIF_tags.")                                                                        # Cam(Seq)++
        else:                                                                                                               # Cam(Seq)++
            image -= dark_level                                                                                             # Cam(Seq)++
            log.ODM_DEBUG("MultiA,  {},  (-= DarkLevel),  {},".format(photo.filename, dark_level))                          # Cam(Seq)++
                                                                                                                            # Cam(Seq)++
            image *= photo.fnumber * photo.fnumber / exposure_time / gain * a1                                              # Cam(Seq)++
            log.ODM_DEBUG("MultiD,  {},  (*= fn*fn/ExposTime/gain * a1),  {},  {},  {},  {},".format(                       # Cam(Seq)++
                    photo.filename, photo.fnumber, exposure_time, gain, a1))                                                # Cam(Seq)++
            return image                                                                                                    # Cam(Seq)++
                                                                                                                            # Cam(Seq)++
    if photo.camera_make == 'DJI' and photo.camera_model == 'FC6360':                                                       # Cam(P4M)++
        dark_level = photo.get_dark_level()                                                                                 # Cam(P4M)++
        exposure_time = photo.exposure_time                                                                                 # Cam(P4M)++
        gain = photo.get_gain()                                                                                             # Cam(P4M)++
        a1, a2, a3 = photo.get_radiometric_calibration()                                                                    # Cam(P4M)++
                                                                                                                            # Cam(P4M)++
        if dark_level is None or exposure_time is None or gain is None or a1 is None:                                       # Cam(P4M)++
            log.ODM_ERROR("Error {} Invalid_EXIF_tags".format(photo.filename))                                              # Cam(P4M)++
            raise RuntimeError("Invalid EXIF tags.")                                                                        # Cam(P4M)++
        else:                                                                                                               # Cam(P4M)++
            image -= dark_level                                                                                             # Cam(P4M)++
            log.ODM_DEBUG("MultiA,  {},  (-= DarkLevel),  {},".format(photo.filename, dark_level))                          # Cam(P4M)++
                                                                                                                            # Cam(P4M)++
            V, x, y = vignette_map(photo)                                                                                   # Cam(P4M)++
                                                                                                                            # Cam(P4M)++
            if x is None:                                                                                                   # Cam(P4M)++
                x, y = np.meshgrid(np.arange(photo.width), np.arange(photo.height))                                         # Cam(P4M)++
                log.ODM_DEBUG("MultiB,  {},  meshgrid=ON".format(photo.filename))                                           # Cam(P4M)++
            else:                                                                                                           # Cam(P4M)++
                log.ODM_DEBUG("MultiB,  {},  meshgrid=None".format(photo.filename))                                         # Cam(P4M)++
                                                                                                                            # Cam(P4M)++
            if V is not None:                                                                                               # Cam(P4M)++
                # vignette correction                                                                                       # Cam(P4M)++
                V = np.repeat(V[:, :, np.newaxis], image.shape[2], axis=2)                                                  # Cam(P4M)++
                image *= V                                                                                                  # Cam(P4M)++
                log.ODM_DEBUG("MultiC,  {},  VignetCorr=ON".format(photo.filename))                                         # Cam(P4M)++
            else:                                                                                                           # Cam(P4M)++
                log.ODM_DEBUG("MultiC,  {},  VignetCorr=None".format(photo.filename))                                       # Cam(P4M)++
                                                                                                                            # Cam(P4M)++
            image *= photo.fnumber * photo.fnumber / exposure_time / gain * a1                                              # Cam(P4M)++
            log.ODM_DEBUG("MultiD,  {},  (*= fn*fn/ExposTime/gain * a1),  {},  {},  {},  {},".format(                       # Cam(P4M)++
                    photo.filename, photo.fnumber, exposure_time, gain, a1))                                                # Cam(P4M)++
            return image                                                                                                    # Cam(P4M)++
                                                                                                                            # Cam(P4M)++
    # All others
    a1, a2, a3 = photo.get_radiometric_calibration()
    dark_level = photo.get_dark_level()

    exposure_time = photo.exposure_time
    gain = photo.get_gain()
    photometric_exp = photo.get_photometric_exposure()

    if a1 is None and photometric_exp is None:
        log.ODM_WARNING("Cannot perform radiometric calibration, no FNumber/Exposure Time or Radiometric Calibration EXIF tags found in %s. Using Digital Number." % photo.filename)
        return image
    
    if a1 is None and photometric_exp is not None:
        a1 = photometric_exp

    V, x, y = vignette_map(photo)
    if x is None:
        x, y = np.meshgrid(np.arange(photo.width), np.arange(photo.height))

    if dark_level is not None:
        image -= dark_level

    # Normalize DN to 0 - 1.0
    bit_depth_max = photo.get_bit_depth_max()
    if bit_depth_max:
        image /= bit_depth_max

    if V is not None:
        # vignette correction
        V = np.repeat(V[:, :, np.newaxis], image.shape[2], axis=2)
        image *= V

    if exposure_time and a2 is not None and a3 is not None:
        # row gradient correction
        R = 1.0 / (1.0 + a2 * y / exposure_time - a3 * y)
        R = np.repeat(R[:, :, np.newaxis], image.shape[2], axis=2)
        image *= R
    
    # Floor any negative radiances to zero (can happen due to noise around blackLevel)
    if dark_level is not None:
        image[image < 0] = 0
    
    # apply the radiometric calibration - i.e. scale by the gain-exposure product and
    # multiply with the radiometric calibration coefficient

    if gain is not None and exposure_time is not None:
        image /= (gain * exposure_time)
    
    image *= a1

    return image

def vignette_map(photo):
    x_vc, y_vc = photo.get_vignetting_center()
    polynomial = photo.get_vignetting_polynomial()

    if x_vc and polynomial:
        # append 1., so that we can call with numpy polyval
        polynomial.append(1.0)
        vignette_poly = np.array(polynomial)

        # perform vignette correction
        # get coordinate grid across image
        x, y = np.meshgrid(np.arange(photo.width), np.arange(photo.height))

        # meshgrid returns transposed arrays
        # x = x.T
        # y = y.T

        # compute matrix of distances from image center
        r = np.hypot((x - x_vc), (y - y_vc))

        # compute the vignette polynomial for each distance - we divide by the polynomial so that the
        # corrected image is image_corrected = image_original * vignetteCorrection

        vignette = 1.0 / np.polyval(vignette_poly, r)
        return vignette, x, y
    
    return None, None, None

# def dn_to_reflectance(photo, image, use_sun_sensor=True):                                                                 # DLS--
def dn_to_reflectance(photo, args, image, use_sun_sensor=True):                                                             # DLS++
    # radiance = dn_to_radiance(photo, image)                                                                               # DLS--
    radiance = dn_to_radiance(photo, args, image)                                                                           # DLS++
    if not use_sun_sensor:                                                                                                  # DLS++
        return radiance                                                                                                     # DLS++
    irrad = get_irradiance(photo)                                                                                           # DLS++
    if irrad is None:                                                                                                       # DLS++
        log.ODM_ERROR("Error {} No_DLS_data_found".format(photo.filename))                                                  # DLS++
        raise RuntimeError("Invalid camera images")                                                                         # DLS++
                                                                                                                            # DLS++
    if photo.camera_make == 'Parrot' and photo.camera_model == 'Sequoia':                                                   # DLS(Seq)++
        if photo.band_name == 'Green':                                                                                      # DLS(Seq)++
            fac = args.radiometric_seq_reflectancefactor_green                                                              # DLS(Seq)++
        elif photo.band_name == 'Red':                                                                                      # DLS(Seq)++
            fac = args.radiometric_seq_reflectancefactor_red                                                                # DLS(Seq)++
        elif photo.band_name == 'Rededge' or photo.band_name == 'RedEdge':                                                  # DLS(Seq)++
            fac = args.radiometric_seq_reflectancefactor_rededge                                                            # DLS(Seq)++
        elif photo.band_name == 'NIR':                                                                                      # DLS(Seq)++
            fac = args.radiometric_seq_reflectancefactor_nir                                                                # DLS(Seq)++
        log.ODM_DEBUG("MultiR,  {},  camera+sun (*= fac/irrad),  {},  {},".format(photo.filename, fac, irrad))              # DLS(Seq)++
        return radiance * fac / irrad                                                                                       # DLS(Seq)++
                                                                                                                            # DLS(Seq)++
    if photo.camera_make == 'DJI' and photo.camera_model == 'FC6360':                                                       # DLS(P4M)++
        if photo.band_name == 'Blue':                                                                                       # DLS(P4M)++
            fac = args.radiometric_p4m_reflectancefactor_blue                                                               # DLS(P4M)++
        elif photo.band_name == 'Green':                                                                                    # DLS(P4M)++
            fac = args.radiometric_p4m_reflectancefactor_green                                                              # DLS(P4M)++
        elif photo.band_name == 'Red':                                                                                      # DLS(P4M)++
            fac = args.radiometric_p4m_reflectancefactor_red                                                                # DLS(P4M)++
        elif photo.band_name == 'Rededge' or photo.band_name == 'RedEdge':                                                  # DLS(P4M)++
            fac = args.radiometric_p4m_reflectancefactor_rededge                                                            # DLS(P4M)++
        elif photo.band_name == 'NIR':                                                                                      # DLS(P4M)++
            fac = args.radiometric_p4m_reflectancefactor_nir                                                                # DLS(P4M)++
        log.ODM_DEBUG("MultiR,  {},  camera+sun (*= fac/irrad),  {},  {},".format(photo.filename, fac, irrad))              # DLS(P4M)++
        return radiance * fac / irrad                                                                                       # DLS(P4M)++
                                                                                                                            # DLS(P4M)++
    irradiance = compute_irradiance(photo, use_sun_sensor=use_sun_sensor)
    return radiance * math.pi / irradiance

def get_irradiance(photo):                                                                                                  # DLS++
    # get radian_solar_elevation, radian_dls_sun                                                                            # DLS++
    ut = photo.get_utc_time()                                                                                               # DLS++
    radian_dls_pose = np.radians(photo.get_dls_pose())                                                                      # DLS++, convert deg. to rad.
    orientation_dls = np.array([0, 0, -1])                                                                                  # DLS++
                                                                                                                            # DLS++
    ned_sun, ned_dls, radian_dls_sun, radian_solar_elevation, solar_azimuth = dls.compute_sun_angle(                        # DLS++
            [photo.latitude, photo.longitude], radian_dls_pose, ut, orientation_dls)                                        # DLS++
                                                                                                                            # DLS++
    # fresnel angle correction                                                                                              # DLS++
    irrad = photo.get_sun_sensor()                                                                                          # DLS++
    corr_angular = dls.fresnel(radian_dls_sun)                                                                              # DLS++
    irrad_dls = irrad / corr_angular                                                                                        # DLS++
    log.ODM_DEBUG("MultiFA, {},  Irr,  {},  (/= CorrAngular),  {},  IrrDLS,  {},".format(                                   # DLS++
            photo.filename, irrad, corr_angular, irrad_dls))                                                                # DLS++
                                                                                                                            # DLS++
    # compute direct irradiance in the plane normal to the sun                                                              # DLS++
    # irrad_dls = irrad_direct * np.cos(radian_dls_sun) + irrad_scattered                                                   # DLS++
    # irrad_dls = irrad_direct * np.cos(radian_dls_sun) + irrad_direct * r_scattered                                        # DLS++
    r_scattered = 1.0 / 6.0     # assumes scattered:direct = 1:6 at clear skies.                                            # DLS++
    irrad_direct = irrad_dls / (np.cos(radian_dls_sun) + r_scattered)                                                       # DLS++
    log.ODM_DEBUG("MultiFB, {},  IrrDLS,  {},  (/= cos(DLS_Sun)+r_scattered),  {},  IrrDirect,  {},".format(                # DLS++
            photo.filename, irrad_dls, np.cos(radian_dls_sun) + r_scattered, irrad_direct))                                 # DLS++
                                                                                                                            # DLS++
    # compute scattered irradiance                                                                                          # DLS++
    irrad_scattered  = irrad_direct * r_scattered                                                                           # DLS++
                                                                                                                            # DLS++
    # compute irradiance on the ground using the solar altitude angle                                                       # DLS++
    irrad_horizontal = irrad_direct * np.sin(radian_solar_elevation) + irrad_scattered                                      # DLS++
    log.ODM_DEBUG("MultiFC, {},  IrrDirect,  {},  IrrHoriz,  {},".format(photo.filename, irrad_direct, irrad_horizontal))   # DLS++
    log.ODM_DEBUG("MultiFD, {},  YPR,  {},  {},  {},  Irr,  {},  degDLS_Sun,  {},  IrrDls,  {},  IrrDirect,  {},  IrrHoriz,  {},".format(   # DLS++
            photo.filename, photo.dls_yaw, photo.dls_pitch, photo.dls_roll, irrad,                                                          # DLS++
            np.degrees(radian_dls_sun), irrad_dls, irrad_direct, irrad_horizontal))                                                         # DLS++
                                                                                                                            # DLS++
    return irrad_horizontal                                                                                                 # DLS++
                                                                                                                            # DLS++
def compute_irradiance(photo, use_sun_sensor=True):
    # Thermal (this should never happen, but just in case..)
    if photo.is_thermal():
        return 1.0

    # Some cameras (Micasense) store the value (nice! just return)
    hirradiance = photo.get_horizontal_irradiance()
    if hirradiance is not None:
        return hirradiance

    # TODO: support for calibration panels

    if use_sun_sensor and photo.get_sun_sensor():
        # Estimate it
        dls_orientation_vector = np.array([0,0,-1])
        sun_vector_ned, sensor_vector_ned, sun_sensor_angle, \
        solar_elevation, solar_azimuth = dls.compute_sun_angle([photo.latitude, photo.longitude],
                                        photo.get_dls_pose(),
                                        photo.get_utc_time(),
                                        dls_orientation_vector)

        angular_correction = dls.fresnel(sun_sensor_angle)

        # TODO: support for direct and scattered irradiance

        direct_to_diffuse_ratio = 6.0 # Assumption, clear skies
        spectral_irradiance = photo.get_sun_sensor()

        percent_diffuse = 1.0 / direct_to_diffuse_ratio
        sensor_irradiance = spectral_irradiance / angular_correction

        # Find direct irradiance in the plane normal to the sun
        untilted_direct_irr = sensor_irradiance / (percent_diffuse + np.cos(sun_sensor_angle))
        direct_irradiance = untilted_direct_irr
        scattered_irradiance = untilted_direct_irr * percent_diffuse

        # compute irradiance on the ground using the solar altitude angle
        horizontal_irradiance = direct_irradiance * np.sin(solar_elevation) + scattered_irradiance
        return horizontal_irradiance
    elif use_sun_sensor:
        log.ODM_WARNING("No sun sensor values found for %s" % photo.filename)
    
    return 1.0

def get_photos_by_band(multi_camera, user_band_name):
    band_name = get_primary_band_name(multi_camera, user_band_name)

    for band in multi_camera:
        if band['name'] == band_name:
            return band['photos']


def get_primary_band_name(multi_camera, user_band_name):
    if len(multi_camera) < 1:
        raise Exception("Invalid multi_camera list")
    
    # multi_camera is already sorted by band_index
    if user_band_name == "auto":
        return multi_camera[0]['name']

    for band in multi_camera:
        if band['name'].lower() == user_band_name.lower():
            return band['name']
    
    band_name_fallback = multi_camera[0]['name']

    log.ODM_WARNING("Cannot find band name \"%s\", will use \"%s\" instead" % (user_band_name, band_name_fallback))
    return band_name_fallback


def compute_band_maps(multi_camera, primary_band):
    """
    Computes maps of: 
     - { photo filename --> associated primary band photo } (s2p)
     - { primary band filename --> list of associated secondary band photos } (p2s)
    by looking at capture UUID, capture time or filenames as a fallback
    """
    band_name = get_primary_band_name(multi_camera, primary_band)
    primary_band_photos = None
    for band in multi_camera:
        if band['name'] == band_name:
            primary_band_photos = band['photos']
            break
    
    # Try using capture time as the grouping factor
    try:
        unique_id_map = {}
        s2p = {}
        p2s = {}

        for p in primary_band_photos:
            uuid = p.get_capture_id()
            if uuid is None:
                raise Exception("Cannot use capture time (no information in %s)" % p.filename)
            
            # Should be unique across primary band
            if unique_id_map.get(uuid) is not None:
                raise Exception("Unreliable UUID/capture time detected (duplicate)")

            unique_id_map[uuid] = p
        
        for band in multi_camera:
            photos = band['photos']

            for p in photos:
                uuid = p.get_capture_id()
                if uuid is None:
                    raise Exception("Cannot use UUID/capture time (no information in %s)" % p.filename)
                
                # Should match the primary band
                if unique_id_map.get(uuid) is None:
                    raise Exception("Unreliable UUID/capture time detected (no primary band match)")

                s2p[p.filename] = unique_id_map[uuid]

                if band['name'] != band_name:
                    p2s.setdefault(unique_id_map[uuid].filename, []).append(p)

        return s2p, p2s
    except Exception as e:
        # Fallback on filename conventions
        log.ODM_WARNING("%s, will use filenames instead" % str(e))

        filename_map = {}
        s2p = {}
        p2s = {}
        file_regex = re.compile(r"^(.+)[-_]\w+(\.[A-Za-z]{3,4})$")

        for p in primary_band_photos:
            filename_without_band = re.sub(file_regex, "\\1\\2", p.filename)

            # Quick check
            if filename_without_band == p.filename:
                raise Exception("Cannot match bands by filename on %s, make sure to name your files [filename]_band[.ext] uniformly." % p.filename)

            filename_map[filename_without_band] = p

        for band in multi_camera:
            photos = band['photos']

            for p in photos:
                filename_without_band = re.sub(file_regex, "\\1\\2", p.filename)

                # Quick check
                if filename_without_band == p.filename:
                    raise Exception("Cannot match bands by filename on %s, make sure to name your files [filename]_band[.ext] uniformly." % p.filename)

                s2p[p.filename] = filename_map[filename_without_band]

                if band['name'] != band_name:
                    p2s.setdefault(filename_map[filename_without_band].filename, []).append(p)

        return s2p, p2s

def compute_alignment_matrices(multi_camera, primary_band_name, images_path, s2p, p2s, max_concurrency=1, max_samples=30):
    log.ODM_INFO("Computing band alignment")

    alignment_info = {}

    # For each secondary band
    for band in multi_camera:
        if band['name'] != primary_band_name:
            matrices = []

            def parallel_compute_homography(p):
                try:
                    if len(matrices) >= max_samples:
                        # log.ODM_INFO("Got enough samples for %s (%s)" % (band['name'], max_samples))
                        return

                    # Find good matrix candidates for alignment
                
                    primary_band_photo = s2p.get(p['filename'])
                    if primary_band_photo is None:
                        log.ODM_WARNING("Cannot find primary band photo for %s" % p['filename'])
                        return

                    warp_matrix, dimension, algo = compute_homography(os.path.join(images_path, p['filename']),
                                                                os.path.join(images_path, primary_band_photo.filename))
                    
                    if warp_matrix is not None:
                        log.ODM_INFO("%s --> %s good match" % (p['filename'], primary_band_photo.filename))

                        matrices.append({
                            'warp_matrix': warp_matrix,
                            'eigvals': np.linalg.eigvals(warp_matrix),
                            'dimension': dimension,
                            'algo': algo
                        })
                    else:
                        log.ODM_INFO("%s --> %s cannot be matched" % (p['filename'], primary_band_photo.filename))
                except Exception as e:
                    log.ODM_WARNING("Failed to compute homography for %s: %s" % (p['filename'], str(e)))

            parallel_map(parallel_compute_homography, [{'filename': p.filename} for p in band['photos']], max_concurrency, single_thread_fallback=False)

            # Find the matrix that has the most common eigvals
            # among all matrices. That should be the "best" alignment.
            for m1 in matrices:
                acc = np.array([0.0,0.0,0.0])
                e = m1['eigvals']

                for m2 in matrices:
                    acc += abs(e - m2['eigvals'])

                m1['score'] = acc.sum()
            
            # Sort
            matrices.sort(key=lambda x: x['score'], reverse=False)
            
            if len(matrices) > 0:
                alignment_info[band['name']] = matrices[0]
                log.ODM_INFO("%s band will be aligned using warp matrix %s (score: %s)" % (band['name'], matrices[0]['warp_matrix'], matrices[0]['score']))
            else:
                log.ODM_WARNING("Cannot find alignment matrix for band %s, The band might end up misaligned!" % band['name'])

    return alignment_info

def compute_homography(image_filename, align_image_filename):
    try:
        # Convert images to grayscale if needed
        image = imread(image_filename, unchanged=True, anydepth=True)
        if image.shape[2] == 3:
            image_gray = to_8bit(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        else:
            image_gray = to_8bit(image[:,:,0])

        max_dim = max(image_gray.shape)
        if max_dim <= 320:
            log.ODM_WARNING("Small image for band alignment (%sx%s), this might be tough to compute." % (image_gray.shape[1], image_gray.shape[0]))

        align_image = imread(align_image_filename, unchanged=True, anydepth=True)
        if align_image.shape[2] == 3:
            align_image_gray = to_8bit(cv2.cvtColor(align_image, cv2.COLOR_BGR2GRAY))
        else:
            align_image_gray = to_8bit(align_image[:,:,0])

        def compute_using(algorithm):
            try:
                h = algorithm(image_gray, align_image_gray)
            except Exception as e:
                log.ODM_WARNING("Cannot compute homography: %s" % str(e))
                return None, (None, None)

            if h is None:
                return None, (None, None)

            det = np.linalg.det(h)
            
            # Check #1 homography's determinant will not be close to zero
            if abs(det) < 0.25:
                return None, (None, None)

            # Check #2 the ratio of the first-to-last singular value is sane (not too high)
            svd = np.linalg.svd(h, compute_uv=False)
            if svd[-1] == 0:
                return None, (None, None)
            
            ratio = svd[0] / svd[-1]
            if ratio > 100000:
                return None, (None, None)

            return h, (align_image_gray.shape[1], align_image_gray.shape[0])
        
        algo = 'feat'
        result = compute_using(find_features_homography)

        if result[0] is None:
            algo = 'ecc'
            log.ODM_INFO("Can't use features matching, will use ECC (this might take a bit)")
            result = compute_using(find_ecc_homography)
            if result[0] is None:
                algo = None
        
        warp_matrix, dimension = result
        return warp_matrix, dimension, algo

    except Exception as e:
        log.ODM_WARNING("Compute homography: %s" % str(e))
        return None, None, (None, None)

def find_ecc_homography(image_gray, align_image_gray, number_of_iterations=1000, termination_eps=1e-8, start_eps=1e-4):
    pyramid_levels = 0
    h,w = image_gray.shape
    min_dim = min(h, w)

    while min_dim > 300:
        min_dim /= 2.0
        pyramid_levels += 1
    
    log.ODM_INFO("Pyramid levels: %s" % pyramid_levels)
    
    # Quick check on size
    if align_image_gray.shape[0] != image_gray.shape[0]:
        align_image_gray = to_8bit(align_image_gray)
        image_gray = to_8bit(image_gray)
        image_gray = cv2.resize(image_gray, None, 
                        fx=align_image_gray.shape[1]/image_gray.shape[1], 
                        fy=align_image_gray.shape[0]/image_gray.shape[0],
                        interpolation=cv2.INTER_AREA)

    # Build pyramids
    image_gray_pyr = [image_gray]
    align_image_pyr = [align_image_gray]

    for level in range(pyramid_levels):
        image_gray_pyr[0] = to_8bit(image_gray_pyr[0], force_normalize=True)
        image_gray_pyr.insert(0, cv2.resize(image_gray_pyr[0], None, fx=1/2, fy=1/2,
                                interpolation=cv2.INTER_AREA))
        align_image_pyr[0] = to_8bit(align_image_pyr[0], force_normalize=True)
        align_image_pyr.insert(0, cv2.resize(align_image_pyr[0], None, fx=1/2, fy=1/2,
                                interpolation=cv2.INTER_AREA))

    # Define the motion model
    warp_matrix = np.eye(3, 3, dtype=np.float32)

    for level in range(pyramid_levels+1):
        ig = gradient(gaussian(image_gray_pyr[level]))
        aig = gradient(gaussian(align_image_pyr[level]))

        if level == pyramid_levels and pyramid_levels == 0:
            eps = termination_eps
        else:
            eps = start_eps - ((start_eps - termination_eps) / (pyramid_levels)) * level
    
        # Define termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                number_of_iterations, eps)

        try:
            log.ODM_INFO("Computing ECC pyramid level %s" % level)
            _, warp_matrix = cv2.findTransformECC(ig, aig, warp_matrix, cv2.MOTION_HOMOGRAPHY, criteria, inputMask=None, gaussFiltSize=9)
        except Exception as e:
            if level != pyramid_levels:
                log.ODM_INFO("Could not compute ECC warp_matrix at pyramid level %s, resetting matrix" % level)
                warp_matrix = np.eye(3, 3, dtype=np.float32)
            else:
                raise e

        if level != pyramid_levels: 
            warp_matrix = warp_matrix * np.array([[1,1,2],[1,1,2],[0.5,0.5,1]], dtype=np.float32)

    return warp_matrix


def find_features_homography(image_gray, align_image_gray, feature_retention=0.25):
    # Detect SIFT features and compute descriptors.
    detector = cv2.SIFT_create(edgeThreshold=10, contrastThreshold=0.1)
    kp_image, desc_image = detector.detectAndCompute(image_gray, None)
    kp_align_image, desc_align_image = detector.detectAndCompute(align_image_gray, None)

    # Match
    bf = cv2.BFMatcher(cv2.NORM_L1,crossCheck=True)
    try:
        matches = bf.match(desc_image, desc_align_image)
    except Exception as e:
        log.ODM_INFO("Cannot match features")
        return None

    # Sort by score
    matches.sort(key=lambda x: x.distance, reverse=False)

    # Remove bad matches
    num_good_matches = int(len(matches) * feature_retention)
    matches = matches[:num_good_matches]

    if len(matches) < 4:
        log.ODM_INFO("Insufficient features: %s" % len(matches))
        return None

    # Debug
    # imMatches = cv2.drawMatches(im1, kp_image, im2, kp_align_image, matches, None)
    # cv2.imwrite("matches.jpg", imMatches)

    # Extract location of good matches
    points_image = np.zeros((len(matches), 2), dtype=np.float32)
    points_align_image = np.zeros((len(matches), 2), dtype=np.float32)

    for i, match in enumerate(matches):
        points_image[i, :] = kp_image[match.queryIdx].pt
        points_align_image[i, :] = kp_align_image[match.trainIdx].pt

    # Find homography
    h, _ = cv2.findHomography(points_image, points_align_image, cv2.RANSAC)
    return h

def gradient(im, ksize=5):
    im = local_normalize(im)
    grad_x = cv2.Sobel(im,cv2.CV_32F,1,0,ksize=ksize)
    grad_y = cv2.Sobel(im,cv2.CV_32F,0,1,ksize=ksize)
    grad = cv2.addWeighted(np.absolute(grad_x), 0.5, np.absolute(grad_y), 0.5, 0)
    return grad

def local_normalize(im):
    width, _ = im.shape
    disksize = int(width/5)
    if disksize % 2 == 0:
        disksize = disksize + 1
    selem = disk(disksize)
    im = rank.equalize(im, selem=selem)
    return im


def align_image(image, warp_matrix, dimension):
    if warp_matrix.shape == (3, 3):
        return cv2.warpPerspective(image, warp_matrix, dimension)
    else:
        return cv2.warpAffine(image, warp_matrix, dimension)


def to_8bit(image, force_normalize=False):
    if not force_normalize and image.dtype == np.uint8:
        return image

    # Convert to 8bit
    try:
        data_range = np.iinfo(image.dtype)
        min_value = 0
        value_range = float(data_range.max) - float(data_range.min)
    except ValueError:
        # For floats use the actual range of the image values
        min_value = float(image.min())
        value_range = float(image.max()) - min_value

    image = image.astype(np.float32)
    image -= min_value
    image *= 255.0 / value_range
    np.around(image, out=image)
    image[image > 255] = 255
    image[image < 0] = 0
    image = image.astype(np.uint8)

    return image


