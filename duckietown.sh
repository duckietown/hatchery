#!/bin/bash

cd $DUCKIETOWN_ROOT
source environment.sh
catkin_make -C catkin_ws/
catkin_ws/devel/setup.bash
echo $DUCKIETOWN_ROOT
cd - && ./gradlew runIde --stacktrace
