# NIPS 2016 package

This directory is a ROS package, link it to your ros_ws/src/ directory

## Launch the process
Bash scripts define the Raspberry Pi as the ROS master, thus its launch file has to be run at first.

### On the Raspberry Pi
```
   # Log in as root via SSH or direct login (required by the node light.py to access the LEDs)
   ssh root@fuzz.local -X
   cd ros_ws
   ./fuzz.local
   roslaunch nips2016 raspberry_pi.launch
```

### On the workstation
The command hereunder will start the services and the controller run:
```
roslaunch nips2016 start.launch name:=new_dataset source:=source_dataset iterations:=100
```


You might want to start only the services to monitor topics and requests services manually, then call instead:
```
roslaunch nips2016 workstation.launch
```
