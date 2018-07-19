#!/bin/bash

ros_distro=kinetic
shell=$(basename $SHELL)
project_root=$1
setup_script=/opt/ros/$ros_distro/setup.$shell

if [ ! -d "$project_root" ]; then
    >&2 echo "Project root $project_root does not exist! Unable to configure."
    exit 1
fi

echo "Setting up ROS environment..."

echo "Activating ROS $ros_distro with shell: $SHELL"
if [ ! -f "$setup_script" ]; then
    >&2 echo "ROS setup script $setup_script does not exist! Unable to configure."
    exit 1
fi

source $setup_script

echo "Building project: $project_root"
catkin_make -C $project_root/catkin_ws/

export PYTHONPATH=$project_root/catkin_ws/src:$PYTHONPATH
echo "Set PYTHONPATH to: $PYTHONPATH"

echo "Activating development environment..."
chmod +x $project_root/catkin_ws/devel/setup.$shell
$project_root/catkin_ws/devel/setup.$shell
