#!/bin/bash
#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# This script cleans the workspace, creates the test results in the desired xml format for Jenkins,
# and uploads the most recent version of the test results to the artifactory

# Cleanup to make sure we have the new testlogs since jenkins sometimes delays this
rm -rf bazel_xml_paths
rm -rf bazel_tests_directory
rm -rf bazel_tests.tar
rm -rf bazel_tests
rm -rf bazel_tests_directory

mkdir bazel_tests_directory
cd bazel-testlogs

# Find the test reports in bazel directory
find . -name "*test.xml" > bazel_xml_paths

# Formatting
sed -i 's/\(.\{2\}\)//' bazel_xml_paths

cd ..
count=0

# Loop to copy the report into desired directory so Jenkins can see them.
while read line; do
    count=$((count + 1))
    rsync -a -L bazel-testlogs/$line bazel_tests_directory/$count/
done <<<$(cat bazel-testlogs/bazel_xml_paths)

# Find missing and existing classnames
grep -iRl "classname" bazel_tests_directory/ > classname_exists
grep -RL "classname" bazel_tests_directory/ > classname_missing

# Loops to format results so that Jenkins plugin can read them
while read line; do
    sed -i 's/classname="/&x86\./g' $line
done <<<$(cat classname_exists)
while read line; do
    sed -i 's/<testcase /&classname="x86\.x86_NoClassFound" /g' $line
done <<<$(cat classname_missing)

# Package and upload to the artifactory
tar -cvf bazel_tests.tar bazel_tests_directory
curl -$1:$2 -T bazel_tests.tar $3
