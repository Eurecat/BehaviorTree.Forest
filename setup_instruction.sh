#!/bin/bash

# build arguments
arg1=${1:-0}

docker build ./Docker -t eut_bt_forest_ws:humble --build-arg="ARG1=$arg1" #--no-cache

if [ ! -d "./Docker/depend" ] 
then
    mkdir ./Docker/depend
fi
cd Docker/depend
git submodule add -b humble git@gitlab.local.eurecat.org:robotics-automation/behavior_tree_eut_plugins.git
git submodule add -b humble git@gitlab.local.eurecat.org:robotics-automation/groot.git
git submodule add -b humble git@gitlab.local.eurecat.org:robotics-automation/behavior_tree_ros.git
git submodule update --init --recursive