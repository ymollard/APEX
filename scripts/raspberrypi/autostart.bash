#!/usr/bin/env bash

ros_master="mingew.local"

#hostname="apex-1-ergo"
hostname=`hostname`
platform_name=`echo $hostname | cut -d '-' -f 1`
platform_id=`echo $hostname | cut -d '-' -f 2`
platform_type=`echo $hostname | cut -d '-' -f 3`

export ROS_MASTER_URI="http://${ros_master}:11311" 
export ROS_HOSTNAME="${hostname}.local"

if [ $platform_type == "torso" ]; then
    roslaunch apex_playground services_torso.launch namespace:=${platform_id}
elif [ $platform_type == "ergo" ]; then
    roslaunch apex_playground services_ergo.launch debug:=false namespace:=${platform_id}
elif [ $platform_type == "distractor" ]; then
    roslaunch apex_playground services_distractor.launch namespace:=${platform_id}
else
    echo -e "\033[0;31mHostname \"${hostname}\" has an unknown type \"${platform_type}\", it should be something like apex-1-ergo!\033[0m"
fi

