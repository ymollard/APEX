# APEX_Playground_sim package

This directory is a ROS package aiming at using simulated Poppy robots for the APEX experiment, by using [V-REP](http://coppeliarobotics.com/).
This is an extension to [the real robot experiment](https://github.com/ymollard/APEX/tree/simulation).
Documentation is poor yet.

## Setting up simulation: Quickstart
You can spawn 1 simulated APEX platform using this command line
```
rosrun apex_playground_sim spawn_simulations.py -n 1 --vrep-path /home/me/bin/V-REP
```
In this command, `-n` specifies how much platforms you wanna spawn, and `--vrep-path` must lead to the root folder of V-REP that you have previously installed.

By default the V-REP GUI pops and the simulation goes real time.
You probably want to multiply simulations by `n>1` instances.
In that case, disabling the V-REP GUI (headless) saves a lot of resources and the simulation will run as fast as possible:

```
rosrun apex_playground_sim spawn_simulations.py -n 6 --vrep-path /home/me/bin/V-REP --headless
```
Same as the real robot experiment, the work manager will pick jobs from [`experiment.json`](https://github.com/ymollard/APEX/blob/simulation/ros/apex_playground/config/experiment.json) and will be served to platforms.

## Notes and tips
In simulation there is N ROS master that goes at their own speed and communicate through ZMQ, while with the real platforms there is a single ROS master (the controller) and platforms can thus communicate through ROS.

A dedicated ROS master is created for each platform starting from port `11311` for platform 1.
Thus there is no need to manually start any roscore. APEX web user interfaces as started from port 5000 for platform 1, e.g.:

* **instance 0** ROS master: http://localhost:11311 APEX UI: http://localhost:5000/
* **instance 1** ROS master: http://localhost:11312 APEX UI: http://localhost:5001/
* **instance 2** ROS master: http://localhost:11313 APEX UI: http://localhost:5002/

You can "connect" to a specific platform by changing your ROS master URI, and perform some operations there

```
ROS_MASTER_URI=http://localhost:11312
rostopic echo /instance_1/iteration    # Will output the current iteration of this platform
rosrun apex_playground_sim get_clock_factor.py   # Will tell the current speed on this master (e.g. x4)
```
