#!/usr/bin/env bash

ros_master="mingew.local"

#hostname="apex-1-ergo"
hostname=`hostname`
platform_name=`echo $hostname | cut -d '-' -f 1`
platform_id=`echo $hostname | cut -d '-' -f 2`
platform_type=`echo $hostname | cut -d '-' -f 3`
ns=$platform_name"_"$platform_id

if [ -f /home/pi/ros_ws/devel_isolated/setup.bash ]; then
 source /home/pi/ros_ws/devel_isolated/setup.bash
fi

export LC_ALL=C # Fix: terminate called after throwing an instance of 'std::runtime_error' what():  locale::facet::_S_create_c_locale name not valid
export ROS_MASTER_URI="http://${ros_master}:11311" 
export ROS_HOSTNAME="${hostname}.local"

if [ $platform_type == "torso" ]; then
    roslaunch apex_playground services_torso.launch namespace:=${ns}
elif [ $platform_type == "ergo" ]; then
    roslaunch apex_playground services_ergo.launch debug:=false namespace:=${ns}
elif [ $platform_type == "distractor" ]; then
    roslaunch apex_playground services_distractor.launch namespace:=${ns}
else
    echo -e "\033[0;31mHostname \"${hostname}\" has an unknown type \"${platform_type}\", it should be something like apex-1-ergo!\033[0m"
fi

