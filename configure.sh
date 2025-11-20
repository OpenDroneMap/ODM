#!/bin/bash

# Ensure the DEBIAN_FRONTEND environment variable is set for apt-get calls
APT_GET="env DEBIAN_FRONTEND=noninteractive $(command -v apt-get)"

check_version(){  
  UBUNTU_VERSION=$(lsb_release -r)
  case "$UBUNTU_VERSION" in
    *"20.04"*|*"21.04"*|*"24.04"*)
      echo "Ubuntu: $UBUNTU_VERSION, good!"
      ;;
    *"18.04"*|*"16.04"*)
      echo "ODM 2.1 has upgraded to Ubuntu 21.04, but you're on $UBUNTU_VERSION"
      echo "* The last version of ODM that supports Ubuntu 16.04 is v1.0.2."
      echo "* The last version of ODM that supports Ubuntu 18.04 is v2.0.0."
      echo "We recommend you to upgrade, or better yet, use docker."
      exit 1
      ;;
    *)
      echo "You are not on Ubuntu 21.04 (detected: $UBUNTU_VERSION)"
      echo "It might be possible to run ODM on a newer version of Ubuntu, however, you cannot rely on this script."
      exit 1
      ;;
  esac
}

if [[ $2 =~ ^[0-9]+$ ]] ; then
    processes=$2
else
    processes=$(nproc)
fi

ensure_prereqs() {
    export DEBIAN_FRONTEND=noninteractive

    if ! command -v sudo &> /dev/null; then
        echo "Installing sudo"
        $APT_GET update
        $APT_GET install -y -qq --no-install-recommends sudo
    else
        sudo $APT_GET update
    fi

    if ! command -v lsb_release &> /dev/null; then
        echo "Installing lsb_release"
        sudo $APT_GET install -y -qq --no-install-recommends lsb-release
    fi

    if ! command -v pkg-config &> /dev/null; then
        echo "Installing pkg-config"
        sudo $APT_GET install -y -qq --no-install-recommends pkg-config
    fi

    echo "Installing tzdata"
    sudo $APT_GET install -y -qq tzdata

    UBUNTU_VERSION=$(lsb_release -r)
    if [[ "$UBUNTU_VERSION" == *"20.04"* ]]; then
        echo "Enabling PPA for Ubuntu GIS"
        sudo $APT_GET install -y -qq --no-install-recommends software-properties-common
        sudo add-apt-repository ppa:ubuntugis/ppa
        sudo $APT_GET update
    elif [[ "$UBUNTU_VERSION" == *"24.04"* ]]; then
        echo "Enabling ubuntugis-unstable PPA for Ubuntu 24.04"
        sudo $APT_GET install -y -qq --no-install-recommends software-properties-common
        sudo add-apt-repository -y ppa:ubuntugis/ubuntugis-unstable
        sudo $APT_GET update
    fi

    echo "Installing Python PIP"
    sudo $APT_GET install -y -qq --no-install-recommends \
        python3-pip \
        python3-setuptools \
        python3-venv
    python3 -m venv venv --system-site-packages
    venv/bin/pip3 install -U pip
    venv/bin/pip3 install -U shyaml
}

# Save all dependencies in snapcraft.yaml to maintain a single source of truth.
# Maintaining multiple lists will otherwise be painful.
installdepsfromsnapcraft() {
    section="$2"
    case "$1" in
        build) key=build-packages; ;;
        runtime) key=stage-packages; ;;
        *) key=build-packages; ;; # shouldn't be needed, but it's here just in case
    esac

    UBUNTU_VERSION=$(lsb_release -r)
    SNAPCRAFT_FILE="snapcraft.yaml"
    if [[ "$UBUNTU_VERSION" == *"21.04"* ]]; then
        SNAPCRAFT_FILE="snapcraft21.yaml"
    elif [[ "$UBUNTU_VERSION" == *"24.04"* ]]; then
        SNAPCRAFT_FILE="snapcraft24.yaml"
    fi

    cat snap/$SNAPCRAFT_FILE | \
        venv/bin/shyaml get-values-0 parts.$section.$key | \
        xargs -0 sudo $APT_GET install -y -qq --no-install-recommends
}

installruntimedepsonly() {
    echo "Installing runtime dependencies"
    ensure_prereqs
    check_version

    echo "Installing Required Requisites"
    installdepsfromsnapcraft runtime prereqs
    echo "Installing OpenCV Dependencies"
    installdepsfromsnapcraft runtime opencv
    echo "Installing OpenSfM Dependencies"
    installdepsfromsnapcraft runtime opensfm
    echo "Installing OpenMVS Dependencies"
    installdepsfromsnapcraft runtime openmvs
}

installreqs() {
    cd /code
    
    ## Set up library paths
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RUNPATH/SuperBuild/install/lib

	## Before installing
    echo "Updating the system"
    ensure_prereqs
    check_version
    
    echo "Installing Required Requisites"
    installdepsfromsnapcraft build prereqs
    echo "Installing OpenCV Dependencies"
    installdepsfromsnapcraft build opencv
    echo "Installing OpenSfM Dependencies"
    installdepsfromsnapcraft build opensfm
    echo "Installing OpenMVS Dependencies"
    installdepsfromsnapcraft build openmvs
    
    set -e

    # edt requires numpy to build
    venv/bin/pip install numpy==2.3.2
    venv/bin/pip install -r requirements.txt --ignore-installed
    #if [ ! -z "$GPU_INSTALL" ]; then
    #fi
    set +e
}
    
install() {
    installreqs

    if [ ! -z "$PORTABLE_INSTALL" ]; then
        echo "Replacing g++ and gcc with our scripts for portability..."
        if [ ! -e /usr/bin/gcc_real ]; then
            sudo mv -v /usr/bin/gcc /usr/bin/gcc_real
            sudo cp -v ./docker/gcc /usr/bin/gcc
        fi
        if [ ! -e /usr/bin/g++_real ]; then
            sudo mv -v /usr/bin/g++ /usr/bin/g++_real
            sudo cp -v ./docker/g++ /usr/bin/g++
        fi
    fi

    set -eo pipefail
    
    echo "Compiling SuperBuild"
    cd ${RUNPATH}/SuperBuild
    mkdir -p build && cd build
    cmake .. && make -j$processes

    echo "Configuration Finished"
}

uninstall() {
    check_version

    echo "Removing SuperBuild and build directories"
    cd ${RUNPATH}/SuperBuild
    rm -rfv build src download install
    cd ../
    rm -rfv build
}

reinstall() {
    check_version

    echo "Reinstalling ODM modules"
    uninstall
    install
}

clean() {
    rm -rf \
        ${RUNPATH}/SuperBuild/build \
        ${RUNPATH}/SuperBuild/download \
        ${RUNPATH}/SuperBuild/src

    # find in /code and delete static libraries and intermediate object files
    find ${RUNPATH} -type f -name "*.a" -delete -or -type f -name "*.o" -delete
}

usage() {
    echo "Usage:"
    echo "bash configure.sh <install|update|uninstall|installreqs|help> [nproc]"
    echo "Subcommands:"
    echo "  install"
    echo "    Installs all dependencies and modules for running OpenDroneMap"
    echo "  installruntimedepsonly"
    echo "    Installs *only* the runtime libraries (used by docker builds). To build from source, use the 'install' command."
    echo "  reinstall"
    echo "    Removes SuperBuild and build modules, then re-installs them. Note this does not update OpenDroneMap to the latest version. "
    echo "  uninstall"
    echo "    Removes SuperBuild and build modules. Does not uninstall dependencies"
    echo "  installreqs"
    echo "    Only installs the requirements (does not build SuperBuild)"
    echo "  clean"
    echo "    Cleans the SuperBuild directory by removing temporary files. "
    echo "  help"
    echo "    Displays this message"
    echo "[nproc] is an optional argument that can set the number of processes for the make -j tag. By default it uses $(nproc)"
}

if [[ $1 =~ ^(install|installruntimedepsonly|reinstall|uninstall|installreqs|clean)$ ]]; then
    RUNPATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    "$1"
else
    echo "Invalid instructions." >&2
    usage
    exit 1
fi
