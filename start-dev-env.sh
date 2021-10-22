#!/bin/bash
set -eo pipefail
__dirname=$(cd $(dirname "$0"); pwd -P)
cd "${__dirname}"

if [ "$1" = "--setup" ]; then
    export HOME=/home/$2

    if [ ! -f .setupdevenv ]; then
        echo "Recompiling environment... this might take a while."
        bash configure.sh reinstall
        
        touch .setupdevenv
        apt update && apt install -y vim git
        chown -R $3:$4 /code
        chown -R $3:$4 /var/www
    fi

    echo "Adding $2 to /etc/passwd"
    echo "$2:x:$3:$4::/home/$2:/bin/bash" >> /etc/passwd
    echo "Adding $2 to /etc/group"
    echo "$2:x:$4:" >> /etc/group
    echo "Adding $2 to /etc/shadow"
    echo "$2:x:14871::::::" >> /etc/shadow
    echo "$2   ALL=(ALL)   NOPASSWD:ALL" >> /etc/sudoers
    echo "odm   ALL=(ALL)   NOPASSWD:ALL" >> /etc/sudoers
    echo "echo '' && echo '' && echo '' && echo '###################################' && echo 'ODM Dev Environment Ready. Hack on!' && echo '###################################' && echo '' && cd /code" > $HOME/.bashrc

    # Install qt creator
    if hash qtcreator 2>/dev/null; then
        has_qtcreator="YES"
    fi

    if [ "$has_qtcreator" != "YES" ] && [ "$5" == "YES" ]; then 
        apt install -y libxrender1 gdb qtcreator
    fi

    # Install liquidprompt
    if [ ! -e "$HOME/liquidprompt" ]; then
        git clone https://github.com/nojhan/liquidprompt.git --depth 1 $HOME/liquidprompt
    fi
    
    if [ -e "$HOME/liquidprompt" ]; then
        echo "source $HOME/liquidprompt/liquidprompt" >> $HOME/.bashrc
        echo "export LP_PS1_PREFIX='(odmdev)'" >> $HOME/.bashrc
    fi

    # Colors
    echo "alias ls='ls --color=auto'" >> $HOME/.bashrc

    # Python paths
    echo $(python3 /code/opendm/context.py) >> $HOME/.bashrc
    
    # Vim 
    printf "syntax on\nset showmatch\nset ts=4\nset sts=4\nset sw=4\nset autoindent\nset smartindent\nset smarttab\nset expandtab" > $HOME/.vimrc

    # Misc aliases
    echo "alias pdal=/code/SuperBuild/install/bin/pdal" >> $HOME/.bashrc
    echo "alias opensfm=/code/SuperBuild/install/bin/opensfm/bin/opensfm" >> $HOME/.bashrc
    

    su -c bash $2
    exit 0
fi

platform="Linux" # Assumed
uname=$(uname)
case $uname in
	"Darwin")
	platform="MacOS / OSX"
	;;
	MINGW*)
	platform="Windows"
	;;
esac

if [[ $platform != "Linux" ]]; then
	echo "This script only works on Linux."
    exit 1
fi

if hash docker 2>/dev/null; then
    has_docker="YES"
fi
if hash nvidia-smi 2>/dev/null; then
    has_nvidia_smi="YES"
fi


if [ "$has_docker" != "YES" ]; then
    echo "You need to install docker before running this script."
    exit 1
fi

IMAGE_SET=NO
if [[ ! -z $IMAGE ]]; then
    IMAGE_SET=YES
fi
export PORT="${PORT:=3000}"
export QTC="${QTC:=NO}"
export IMAGE="${IMAGE:=opendronemap/nodeodm}"
export GPU="${GPU:=NO}"

if [ -z "$DATA" ]; then
    echo "Usage: DATA=/path/to/datasets [VARS] $0"
    echo
    echo "VARS:"
    echo "	DATA	Path to directory that contains datasets for testing. The directory will be mounted in /datasets. If you don't have any, simply set it to a folder outside the ODM repository."
    echo "	PORT	Port to expose for NodeODM (default: $PORT)"
    echo "	IMAGE	Docker image to use (default: $IMAGE)"
    echo "	GPU	Enable GPU support (default: $GPU)"
    echo "	QTC	When set to YES, installs QT Creator for C++ development (default: $QTC)"
    exit 1
fi


echo "Starting development environment..."
echo "Datasets path: $DATA"
echo "Expose port: $PORT"
echo "QT Creator: $QTC"
echo "Image: $IMAGE"
echo "GPU: $GPU"

if [ ! -e "$HOME"/.odm-dev-home ]; then
    mkdir -p "$HOME"/.odm-dev-home
fi

USER_ID=$(id -u)
GROUP_ID=$(id -g)
USER=$(id -un)
GPU_FLAGS=""
if [[ "$GPU" != "NO" ]]; then
    if [[ "$IMAGE_SET" = "NO" ]]; then
        IMAGE="$IMAGE:gpu"
    fi

    GPU_FLAGS="--gpus all"
    if [[ "$has_nvidia_smi" = "YES" ]]; then
        GPU_FLAGS="$GPU_FLAGS --device /dev/nvidia0 --device /dev/nvidia-uvm --device /dev/nvidia-uvm-tools --device /dev/nvidia-modeset --device /dev/nvidiactl"
    fi
fi

xhost + || true
docker run -ti --entrypoint bash --name odmdev --user root -v $(pwd):/code -v "$DATA":/datasets -p $PORT:3000 $GPU_FLAGS --privileged -e DISPLAY -e LANG=C.UTF-8 -e LC_ALL=C.UTF-8 -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v="$HOME/.odm-dev-home:/home/$USER" $IMAGE -c "/code/start-dev-env.sh --setup $USER $USER_ID $GROUP_ID $QTC"
exit 0
