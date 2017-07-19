#!/bin/bash

master_hostname="ros.local"
hostname=`hostname`

env_vars="export ROS_MASTER_URI=\"http://${master_hostname}:11311\" 
export ROS_HOSTNAME=\"${hostname}.local\" 
export PS1=\"\[\033[00;33m\][\${ROS_MASTER_URI}]\[\033[00m\] \${PS1}\""

if [ -f /home/pi/ros_ws/devel_isolated/setup.bash ]; then
 source /home/pi/ros_ws/devel_isolated/setup.bash
fi

if [ -f /home/pi/ros_ws/devel/setup.bash ]; then
 source /home/pi/ros_ws/devel/setup.bash
fi

bash --rcfile <(cat ~/.bashrc; echo "${env_vars}")
