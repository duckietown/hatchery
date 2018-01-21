#!/bin/sh

project_root=$(echo "$@" | sed 's/^.*-Project=//')
echo "Configuring project root: $project_root"

if [ ! -d "$project_root" ]; then
    >&2 echo "Project root does not exist! Unable to configure."
    exit 1
fi

cd $project_root
source environment.sh
catkin_make -C catkin_ws/
catkin_ws/devel/setup.bash
cd -