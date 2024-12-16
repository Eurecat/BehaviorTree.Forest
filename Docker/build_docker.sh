#!/bin/bash

# build arguments
arg1=${1:-0}

docker build . -t eut_bt_ros2:humble --build-arg="ARG1=$arg1" --no-cache