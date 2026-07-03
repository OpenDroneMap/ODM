#!/usr/bin/env python3
"""Smoke-test an ODM install: the CLI runs and core libraries import.
"""
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def check_openmvs_loads():
    # A missing shared library aborts the binary before it prints its banner.
    omvs = os.path.join(ROOT, "SuperBuild", "install", "bin", "OpenMVS")
    suffix = ".exe" if sys.platform == "win32" else ""
    for name in ("DensifyPointCloud", "ReconstructMesh"):
        proc = subprocess.run(
            [os.path.join(omvs, name + suffix), "--help"],
            capture_output=True, text=True,
        )
        if "OpenMVS" not in proc.stdout:
            sys.exit("smoke failed: %s did not load" % name)


def main():
    for tool in ("pdal", "untwine"):
        if shutil.which(tool) is None:
            sys.exit("smoke failed: required executable not found on PATH: %s" % tool)

    check_openmvs_loads()

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
