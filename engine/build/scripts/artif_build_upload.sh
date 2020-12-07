#!/bin/bash
#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# This script builds all the tests for Jetson devices and uploads all binaries and dependencies to the artifactory.

# constants
BAZEL_BIN="bazel-bin"
LOCAL_DEVICE="x86_64"

#clean the directory from previous runs and create the necessary files & folders
rm -rf executables/
rm -rf test_dep/
rm -rf bazel-bin-copy/
rm -rf jetson_artifactory
rm jetson_tests.tar
rm -rf jetson_testlog
>all_tests
>remote_tests
>dependencies_test
>dep_loc
>dep_loc_tmp
>exe_loc_tmp
>exe_tmp
>py_dep
mkdir test_dep
mkdir executables
mkdir bazel-bin-copy

# Helper functions to be used in this script.
# Prints the error message and exits the script.
error_and_exit() {
    printf "Error: $1\n"
    exit 1
}

# Prints the warning message.
warn() {
    printf "Warning: $1\n"
}

# used arguments with default values
UNAME=$USER
REMOTE_USER=nvidia #if no username provided
REMOTE_USER_SET=false

# get command line arguments
while [ $# -gt 0 ]; do
    case "$1" in
    -d | --device)
        DEVICE="$2"
        ;;
    -h | --host)
        HOST="$2"
        ;;
    -u | --user)
        UNAME="$2"
        ;;
    -a | --artifactory)
        ARTIF="$2"
        ;;
    -p | --pword)
        PWORD="$2"
        ;;
    -r | --run)
        NEED_RUN="True"
        shift
        continue
        ;;
    --remote_user)
        REMOTE_USER="$2"
        REMOTE_USER_SET=true
        ;;
    --deploy_path)
        DEPLOY_PATH="$2"
        ;;
    *)
        printf "Error: Invalid arguments: %1 %2\n"
        exit 1
        ;;
    esac
    shift
    shift
done

if [ -z "$DEVICE" ]; then
    error_and_exit "Desired target device must be specified with -d DEVICE. Valid choices: 'jetpack43', 'x86_64'."
fi

# find all the tests that are written for the given configuration, except for the ones tagged "host", "manual", "pytest", "performance"
bazel cquery 'kind(".*_test rule", ...)' --config $DEVICE --build_tag_filters=-host,-manual,-pytest,-performance>>all_tests

# string manipulation to feed the tests to the bazel build
sed -i 's/\s.*$//' ./all_tests

# remove the tests that we do not want to run on jetson
sed -i '/cpplint/d' ./all_tests
sed -i '/_check_json/d' ./all_tests

tests_to_build=$(cat all_tests)

echo "================================================================================"
echo "Building for target platform '$DEVICE'"
echo "================================================================================"

# build everything for the given configuration except for the ones tagged "host", "manual", "pytest", "performance"
bazel build --config $DEVICE --build_tag_filters=-host,-manual,-pytest-performance $tests_to_build

# string formatting
sed -r -i 's/:/\//g' all_tests
cut -c 3- all_tests > all_tests_formatted

cat all_tests_formatted | while read line; do
    # paths to the test targets are stored in the $line variable now
    # find the dependencies for the tests and save them in a file to read later
    objdump -x -p bazel-bin/$line | grep NEEDED >>dependencies_test
done

# remove the duplicate dependencies
awk '!x[$0]++' dependencies_test > uniq_dep_tmp

# formatting
cut -c 24- uniq_dep_tmp > uniq_dep

cat uniq_dep | while read line; do
    # paths to the test targets are stored in the $line variable now
    # the bazel cache location might differ, need to figure out where it is for the docker image this script is being run on
    dep_path=$(find $HOME/.cache/bazel -name "$line" -print -quit | head -1)

    # check if the found path is to a file
    # if not, check the other library path where the system stores them
    if [[ -f $dep_path ]]; then
        :
    else
        dep_path=$(find /usr/aarch64-linux-gnu/lib/ -name "$line" -print -quit | head -1)
    fi

    # if a file is found, add to the location list
    if [[ -f $dep_path ]]; then
        echo $dep_path >>dep_loc_tmp
    else
        echo "$line NOT FOUND"
    fi
done

# remove the duplicate dependencies
awk '!x[$0]++' dep_loc_tmp > dep_loc

cat dep_loc | while read line; do
    # paths to the dependencies are stored in the $line variable now
    # copy all the dependency files into one directory
    rsync -a -L $line $PWD/test_dep
done

# since bazel-bin is a symlink itself, and we will not copy symlinks after we create a copy directory of bazel-bin where the build artifacts are stored
rsync -a $PWD/bazel-bin/ bazel-bin-copy/

echo "================================================================================"
echo "Deploying all tests to the artifactory"
echo "================================================================================"

# copy to the target device, excluding the files & directories that are not needed for tests to run
rsync -a -l --exclude '*.lo' --exclude '*.a' --exclude '*.d' --exclude '*.o' --exclude 'bazel-genfiles' --exclude 'bazel-bin' --exclude 'bazel-isaac' --exclude 'bazel-isaac-public' --exclude 'bazel-out' --exclude 'bazel-testlogs' --exclude 'bazel-workspace' $PWD $PWD/jetson_artifactory
tar -cvf jetson_tests.tar jetson_artifactory
curl -$UNAME:$PWORD -T jetson_tests.tar $ARTIF

echo "================================================================================"
echo "Cleaning up"
echo "================================================================================"

rm -rf excectutables/
rm -rf executables/
rm -rf test_dep/
rm -rf bazel-bin-copy/
rm all_tests
rm all_tests_formatted
rm remote_tests
rm dependencies_test
rm dep_loc
rm dep_loc_tmp
rm exe_loc_tmp
rm exe_tmp
rm py_dep
rm uniq_dep
rm uniq_dep_tmp
rm -rf jetson_artifactory