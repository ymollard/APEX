#!/bin/bash

source devel/setup.bash

master_hostname="apex-controller.local"
hostname=`hostname`

env_vars="export ROS_MASTER_URI=\"http://${master_hostname}:11311\" 
export ROS_HOSTNAME=\"${hostname}.local\" 
export PS1=\"\[\033[00;33m\][\${ROS_MASTER_URI}]\[\033[00m\] \${PS1}\""

bash --rcfile <(cat ~/.bashrc; echo "${env_vars}")

