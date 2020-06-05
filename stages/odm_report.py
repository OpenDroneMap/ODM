import os

from opendm import log
from opendm import io
from opendm import system
from opendm import types
from opendm.shots import get_geojson_shots_from_opensfm
import json


class ODMReport(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        if not os.path.exists(tree.odm_report): system.mkdir_p(tree.odm_report)

        shots_geojson = os.path.join(tree.odm_report, "shots.geojson")
        if not io.file_exists(shots_geojson) or self.rerun():
            # Extract geographical camera shots
            if reconstruction.is_georeferenced():
                shots = get_geojson_shots_from_opensfm(tree.opensfm_reconstruction, tree.opensfm_transformation, reconstruction.get_proj_srs())    
            else:
                # Pseudo geo
                shots = get_geojson_shots_from_opensfm(tree.opensfm_reconstruction, pseudo_geotiff=tree.odm_orthophoto_tif)

            if shots:
                with open(shots_geojson, "w") as fout:
                    fout.write(json.dumps(shots))

                log.ODM_INFO("Wrote %s" % shots_geojson)
            else:
                log.ODM_WARNING("Cannot extract shots")
        else:
            log.ODM_WARNING('Found a valid shots file in: %s' % shots_geojson)
