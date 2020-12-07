#!/bin/bash
#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# script to upload a single app container to NCG

# fail on error

set -o errexit -o pipefail -o noclobber -o nounset

# default values

REPOSITORY="nvstaging/isaac-validation"
KEEP=false
USER='$oauthtoken'
PASSWORD=''

function show_help {
    echo "Usage: $0 [ -r <repository> ] <container-name> [ <container-name> ... ]"

    echo "Arguments:"
    echo "  -r <repository>     Repository to upload to, default: '${REPOSITORY}'"
    echo "  -k                  Keep the container locally (default: trash the container after upload)"
    echo "  <container-name>    Tag of the container to upload"

    exit 1
}

# get command line arguments
OPTIONS=r:khu:p:
LONGOPTS=repository:,keep,help,user:,password:
PARSED=$(getopt --options="${OPTIONS}" --longoptions="${LONGOPTS}" --name "$0" -- "$@")
eval set -- "${PARSED}"

while true
do
    case "$1" in
        -r|--repository)
            REPOSITORY="$2"
            shift 2
            ;;
        -u|--user)
            USER="$2"
            shift 2
            ;;
        -p|--password)
            PASSWORD="$2"
            shift 2
            ;;
        -k|--keep)
            KEEP=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "$0: Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Make sure we log in

echo "$PASSWORD" | docker login -u "${USER}" --password-stdin nvcr.io

# Upload containers one at a time

CONTAINERS=("$@")

for CONTAINER in "${CONTAINERS[@]}"
do
    CONTAINERBASE="${CONTAINER%:*}"

    REMOTETAG="nvcr.io/${REPOSITORY}/${CONTAINER}"
    LATESTTAG="nvcr.io/${REPOSITORY}/${CONTAINERBASE}:latest"

    docker tag "${CONTAINER}" "${REMOTETAG}"
    docker tag "${CONTAINER}" "${LATESTTAG}"

    docker push "${REMOTETAG}"
    docker push "${LATESTTAG}"

    if [ ! $KEEP ]
    then
        docker rmi "${CONTAINER}"
        docker rmi "${REMOTETAG}"
        docker rmi "${LATESTTAG}"
    fi
done
