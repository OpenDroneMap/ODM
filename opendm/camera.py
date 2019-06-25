import os, json
from opendm import log

def get_cameras_from_opensfm(reconstruction_file):
    """
    Extract the cameras from OpenSfM's reconstruction.json
    """
    if os.path.exists(reconstruction_file):
        with open(reconstruction_file, 'r') as fin:
            reconstructions = json.loads(fin.read())
            
            result = {}
            for recon in reconstructions:
                if 'cameras' in recon:
                    for camera_id in recon['cameras']:
                        # Strip "v2" from OpenSfM camera IDs
                        new_camera_id = camera_id
                        if new_camera_id.startswith("v2 "):
                            new_camera_id = new_camera_id[3:]

                        result[new_camera_id] = recon['cameras'][camera_id]
                        
                        # Remove "_prior" keys
                        keys = list(result[new_camera_id].keys())
                        for k in keys:
                            if k.endswith('_prior'):
                                result[new_camera_id].pop(k)
            return result
    else:
        raise RuntimeError("%s does not exist." % reconstruction_file)


def get_opensfm_camera_models(cameras):
    """
    Convert cameras to a format OpenSfM can understand
    (opposite of get_cameras_from_opensfm)
    """
    if isinstance(cameras, dict):
        result = {}
        for camera_id in cameras:
            # Quick check on IDs
            if len(camera_id.split(" ")) < 6:
                raise RuntimeError("Invalid cameraID: %s" % camera_id)

            # Add "v2" to camera ID
            if not camera_id.startswith("v2 "):
                osfm_camera_id = "v2 " + camera_id
            else:
                osfm_camera_id = camera_id
            
            # Add "_prior" keys
            camera = cameras[camera_id]
            prior_fields = ["focal","focal_x","focal_y","c_x","c_y","k1","k2","p1","p2","k3"]
            valid_fields = ["id","width","height","projection_type"] + prior_fields + [f + "_prior" for f in prior_fields]

            keys = list(camera.keys())
            for param in keys:
                param_prior = param + "_prior"
                if param in prior_fields and not param_prior in camera:
                    camera[param_prior] = camera[param]

            # Remove invalid keys
            keys = list(camera.keys())
            for k in keys:
                if not k in valid_fields:
                    camera.pop(k)
                    log.ODM_WARNING("Invalid camera key ignored: %s" % k)

            result[osfm_camera_id] = camera
        return result
    else:
        raise RuntimeError("Invalid cameras format: %s. Expected dict." % str(cameras))
