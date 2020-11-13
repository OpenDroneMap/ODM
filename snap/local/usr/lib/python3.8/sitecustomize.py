import site
import os

snap_dir = os.getenv("SNAP")
snapcraft_stage_dir = os.getenv("SNAPCRAFT_STAGE")
snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")

# Do not include snap_dir during builds as this will include
# snapcraft's in-snap site directory.
if snapcraft_stage_dir is not None and snapcraft_part_install is not None:
    site_directories = [snapcraft_stage_dir, snapcraft_part_install]
else:
    superbuild_dir = os.path.join(snap_dir, 'odm/SuperBuild/install')
    site_directories = [snap_dir, superbuild_dir]

for d in site_directories:
    if d:
        site_dir = os.path.join(d, "lib/python3.8/site-packages")
        site.addsitedir(site_dir)

if snap_dir:
    site.ENABLE_USER_SITE = False
