# APEX playground V-Rep simulation

## V-Rep setup

```
# Edit the V-Rep remote API configuration to activate several servers and add as many connections as needed:
nano V-REP_PRO_EDU_V3_3_2_64_Linux/remoteApiConnections.txt

# Compile the remoteApi
cd V-REP_PRO_EDU_V3_3_2_64_Linux/programming/remoteApiBindings/lib
nano makefile # Replace the non-standardized 4 spaces for tabs 
make

cd ../python/python/
ln -s ../../../../programming/remoteApiBindings/lib/lib/64Bit/remoteApi.so

# Enable the ROS interface
cd V-REP_PRO_EDU_V3_3_2_64_Linux/
ln -s compiledRosPlugins/libv_repExtRosInterface.so
# Attach the LUA script [`clock_publisher.lua`](clock_publisher.lua) to any object (normally already present in the ttt)
# ROS param `/use_sim_time` must be set
# http://www.coppeliarobotics.com/helpFiles/en/rosTutorialIndigo.htm

# Start V-Rep now, check the debug trace whether it's starting the requested servers
python simpleTest.py
# In case of failure, check that this port is allowed in remoteApiConnections.txt

# Load the APEX ttt (IMPORTANT) and start the simulation, then connect the controllers
roslaunch poppy_ergo_jr_controllers controllers.launch simulated:=true
```



## Pypot install
```
sudo pip install "pypot<3"
sudo pip install "poppy-creature<2"
```
