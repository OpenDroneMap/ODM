#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

bash "${ROOT}/run.sh" --help >/dev/null
python3 -c "from opensfm import io, pymap; from osgeo import gdal; import cv2; import pdal; print('smoke ok', gdal.__version__, cv2.__version__, pdal.__version__)"
command -v pdal >/dev/null
command -v untwine >/dev/null
