#!/bin/bash

ros_distro=kinetic
shell=$(basename $SHELL)
project_root=$(echo "$@" | grep "\-Project=" | sed 's/^.*-Project=//')

if [ -z "$project_root" ]; then
    project_root="$DUCKIETOWN_ROOT"
fi

if [ ! -d "$project_root" ]; then
    >&2 echo "Project root does not exist! Unable to configure."
    exit 1
fi

echo "Setting up ROS environment..."

echo "Activating ROS with shell: $SHELL"
source /opt/ros/$ros_distro/setup.$shell

echo "Building project: $project_root"
catkin_make -C $project_root/catkin_ws/

export PYTHONPATH=$project_root/catkin_ws/src:$PYTHONPATH
echo "Set PYTHONPATH to: $PYTHONPATH"

echo "Activating development environment..."
chmod +x $project_root/catkin_ws/devel/setup.$shell
$project_root/catkin_ws/devel/setup.$shell
