#!/bin/bash

set -e

if [ $# -lt 1 ]; then
    echo 'usage: input_folder [needs to contain an images directory]'
    exit
fi

input_folder=$1

if [ ${input_folder:0:1} == "-" ]; then
    echo 'usage: input_folder [needs to contain an images directory]'
    exit
fi

echo ${input_folder}

docker container run -it --rm \
    -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro --user $(id -u):$(id -g) \
    -v "${input_folder}":/datasets \
    local-odm \
    --project-path /datasets \
    .
