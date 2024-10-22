#!/usr/bin/env bash

export ROS_MASTER_URI=http://192.168.12.20:11311
export ROS_IP=192.168.12.21

source /opt/ros/noetic/setup.bash
source /home/$USER/catkin_ws/devel/setup.bash
source /home/$USER/catkin_isolated_ws/devel_isolated/setup.bash
source /home/$USER/vistec_waiter_ws/devel/setup.bash
exec "$@"
