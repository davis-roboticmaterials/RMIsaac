#!/bin/bash
#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# This script pulls the necessary files for running the tests on Jetson from the Artifactory and the runs all the tests.
# After tests are done running it reports the results back.

rm -rf jetson_artifactory
rm -rf $WORKSPACE/jetson_testlog
rm -rf jetson_testlog
curl -$1:$2 -O $3
tar -xvf $4
rm -rf $4
cd jetson_artifactory
for dir in */; do cd "$dir"; done
rm -rf jetson_testlog
rm -rf aborted_testlog
rm -rf aborted_tests.xml
mkdir jetson_testlog
rm -rf jetson-test-out.txt
export LD_LIBRARY_PATH=$PWD/test_dep
total_aborted=0

#Load the error message template, in case a test gets aborted
ERR1='' read -r -d '' String1 <<"EOF"
<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="1" disabled="0" errors="0" time="0" name="AllTests">
  <testsuite name="insert_test_suite" tests="1" failures="1" disabled="0" errors="0" time="0">
    <testcase name="insert_test_case" status="run" time="0" classname="insert_test_suite">
      <failure message="TEST ABORTED. SEE STACK TRACE FOR TERMINAL OUTPUT:" type=""><![CDATA[
EOF
ERR2='' read -r -d '' String2 <<"EOF"
]]></failure>
    </testcase>
  </testsuite>
</testsuites>
EOF

while read line; do
    # addresses of the test directories are stored in the $line variable now
    echo "================================================================================"
    echo "RUNNING $line"
    echo "================================================================================"
    # list the tests here and then run them individually --gtest_list_tests
    ./bazel-bin-copy/$line --gtest_list_tests |& tee test_cases
    test_first=$(head -n 1 test_cases)
    if [ "$test_first" == "Running main() from gtest_main.cc" ]; then
        tail -n +2 "test_cases" > "test_cases.tmp" && mv "test_cases.tmp" "test_cases"
    fi
    while read test_case_line; do
        last_char=${test_case_line: -1}
        if [ "$last_char" == "." ]; then
            test_suite=${test_case_line}
        else
            test_case="${test_suite}${test_case_line}"
            counter_before=$(ls -1q jetson_testlog | wc -l)
            timeout 120 ./bazel-bin-copy/$line --gtest_color=no --gtest_filter="$test_case" --gtest_output="xml:jetson_testlog/" |& tee jetson-test-out.txt
            counter_after=$(ls -1q jetson_testlog | wc -l)
            if [ "$counter_before" -eq "$counter_after" ]; then
                total_aborted=$((total_aborted + 1))
                msg=$(<jetson-test-out.txt)
                echo -e "${String1}\n${msg}\n${String2}" > $test_case.xml
                sed -i "s/insert_test_suite/${test_suite::-1}/g" $test_case.xml
                sed -i "s/insert_test_case/$test_case_line/g" $test_case.xml
                sed -i 's/\x1b\[[0-9;]*m//g' $test_case.xml
                cp $test_case.xml $PWD/jetson_testlog
                rm -rf $test_case.xml
            fi
        fi
    done <<<$(cat $PWD/test_cases)
done <<<$(cat $PWD/all_tests_formatted)

# change all the classnames here to Jetson.<classname>, to seperate from bazel tests
for file in $PWD/jetson_testlog/*
do
    sed -i 's/classname="/&Jetson\./g' $file
done

# copy test reuslts to jenkins workspace so the pipeline can access them
cp -r jetson_testlog $WORKSPACE/jetson_testlog