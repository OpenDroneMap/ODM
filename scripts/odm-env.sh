# Runtime environment for ODM (pixi/conda). Source from run.sh, test.sh, etc.

_odm_env_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${CONDA_PREFIX:-}" ]; then
    echo "ODM requires a pixi environment. Run: pixi install && pixi run <task>" >&2
    exit 1
fi

export PATH="${CONDA_PREFIX}/bin:${PATH}"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${_odm_env_root}/SuperBuild/install/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export DYLD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${_odm_env_root}/SuperBuild/install/lib${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"

if [ -d "${CONDA_PREFIX}/share/proj" ]; then
    export PROJ_LIB="${CONDA_PREFIX}/share/proj"
fi
if [ -d "${CONDA_PREFIX}/share/gdal" ]; then
    export GDAL_DATA="${CONDA_PREFIX}/share/gdal"
fi

if [ -d "${CONDA_PREFIX}/lib/pdal/plugins" ]; then
    export PDAL_DRIVER_PATH="${CONDA_PREFIX}/lib/pdal/plugins"
elif [ -d "${CONDA_PREFIX}/Library/lib/pdal/plugins" ]; then
    export PDAL_DRIVER_PATH="${CONDA_PREFIX}/Library/lib/pdal/plugins"
else
    export PDAL_DRIVER_PATH="${_odm_env_root}/SuperBuild/install/bin"
fi

eval "$(cd "${_odm_env_root}" && python3 -c "import opendm.context; print('export PYTHONPATH=' + ':'.join(opendm.context.python_packages_paths))")"
