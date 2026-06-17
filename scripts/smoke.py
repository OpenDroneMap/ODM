#!/usr/bin/env python3
"""Smoke-test an ODM install: the CLI runs and core libraries import.

Cross-platform replacement for the old smoke.sh/smoke.bat pair. Invoked via
`pixi run smoke`; in the Docker runtime image it is launched against the
activated prod env via `docker-entrypoint.sh python3 scripts/smoke.py`.
"""
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def main():
    for tool in ("pdal", "untwine"):
        if shutil.which(tool) is None:
            sys.exit("smoke failed: required executable not found on PATH: %s" % tool)

    # Importing opendm.context wires up the opensfm sys.path and, on Windows,
    # the DLL search dirs + GDAL/PROJ/PDAL env that the imports below need.
    import opendm.context  # noqa: F401

    subprocess.run(
        [sys.executable, os.path.join(ROOT, "run.py"), "--help"],
        check=True, stdout=subprocess.DEVNULL,
    )

    from opensfm import io, pymap  # noqa: F401
    from osgeo import gdal
    import cv2
    import pdal

    print("smoke ok", gdal.__version__, cv2.__version__, pdal.__version__)


if __name__ == "__main__":
    main()
