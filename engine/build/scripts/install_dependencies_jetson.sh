#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

set -o errexit -o pipefail -o noclobber -o nounset

UNAME=nvidia

### default arguments

# whether this is a local installation, i.e. it is executed directly on the device, or in a
# cross-platform docker container, default is to ssh to a remote host
LOCAL_INSTALL=false

# for remote install, host name or IP to connect to
HOST=""

# get command line arguments
while [ $# -gt 0 ]
do
  case "$1" in
    -h|--host)
      HOST="$2"
      shift 2
      ;;
    -l|--local)
      LOCAL_INSTALL=true
      shift
      ;;
    -u|--user)
      UNAME="$2"
      shift 2
      ;;
    *)
      printf "Error: Invalid arguments: %1 %2\n"
      exit 1
  esac
done

if [ -z "${HOST}" -a "${LOCAL_INSTALL}" = false ]
then
  echo "Error: Jetson device IP must be specified with -h IP."
  exit 1
fi

# This function will be ran on the target device
remote_function() {
  local IS_LOCAL="$1"

  # Install packages
  sudo apt update && sudo apt install -y rsync curl libhidapi-libusb0\
       libturbojpeg python3-pip python3-numpy redis-server

  # pycapnp is required to run Python Codelets.
  echo "Installing pycapnp. This may take about five minutes to complete. Please wait..."
  python3 -m pip install --user pycapnp

  # psutil is needed for navigation evaluation
  python3 -m pip install psutil

  # packages needed for jupyter notebooks
  echo "Installing jupyterlab. This may take a while to complete. Please wait..."
  sudo apt install -y libjpeg-dev libfreetype6-dev
  python3 -m pip install --user cython
  python3 -m pip install --user numpy jupyterlab matplotlib Pillow

  # Dependencies for engine
  python3 -m pip install --user numpy-quaternion

  # Dependency for engine/pyalice/gui/composite_widget.py
  python3 -m pip install --user ipywidgets
  jupyter nbextension enable --py widgetsnbextension

  if [ "${IS_LOCAL}" = false ]
  then
    # Give user permission to use i2c and tty devices
    sudo usermod -a -G i2c,dialout $USER

    # Blacklist nvs_bmi160 kernel mod
    echo blacklist nvs_bmi160 | sudo tee /etc/modprobe.d/blacklist-nvs_bmi160.conf > /dev/null

    sudo nohup bash -c "sleep 5; shutdown -r now" &
  fi
}

if [ "${LOCAL_INSTALL}" = true ]
then
  remote_function "${LOCAL_INSTALL}"
else
  # Installs dependencies on Jetson devices
  ssh -t $UNAME@$HOST "$(declare -pf remote_function); remote_function false"

  echo "Rebooting the Jetson device"
fi

