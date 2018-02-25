#!/bin/bash

project_root=$(echo "$@" | grep "\-Project=" | sed 's/^.*-Project=//')

if [ -z "$project_root" ]; then
    project_root="$DUCKIETOWN_ROOT"
fi

if [ ! -d "$project_root" ]; then
    >&2 echo "Project root does not exist! Unable to configure."
    exit 1
fi

echo "Configuring project root: $project_root"

cd $project_root

if [ -z "$HOSTNAME" ]; then
    echo "Need to set HOSTNAME."
    exit 1
fi

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp

shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/lunar/setup.$shell

export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME=$HOSTNAME.local
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"

export DUCKIETOWN_ROOT=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
echo "Set DUCKIETOWN_ROOT to: $DUCKIETOWN_ROOT"

export PYTHONPATH=$DUCKIETOWN_ROOT/catkin_ws/src:$PYTHONPATH
echo "Set PYTHONPATH to: $PYTHONPATH"

echo "Activating development environment..."
source $DUCKIETOWN_ROOT/catkin_ws/devel/setup.$shell
catkin_make -C catkin_ws/
catkin_ws/devel/setup.bash

cd -
