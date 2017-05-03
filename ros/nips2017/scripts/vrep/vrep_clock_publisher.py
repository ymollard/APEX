#!/usr/bin/env python
import pypot.vrep.remoteApiBindings.vrep as vrep # Pypot has the path to V-Rep bindings, but that could also be a direct import
import rospy
from rosgraph_msgs.msg import Clock


class VRepClockPublisher(object):
    """
    This is a loop publishing VRep time to ROS at 20Hz
    It owns the the main VRep Remote API port and opens the other ports for other nodes
    """
    def __init__(self):
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        self.simulation_id = -1

    def run(self, initial_vrep_port):
        self.simulation_id = vrep.simxStart('127.0.0.1', initial_vrep_port, True, False, 5000, 5)
        while not rospy.is_shutdown():
            # We loop as fast as possible, in any case V-Rep can't stream more than 20Hz
            # We're surely loosing a bunch of ms here...
            vrep.simxGetPingTime(self.simulation_id)  # Just to force V-Rep updating its simulation time
            vrep_time = vrep.simxGetLastCmdTime(self.simulation_id)
            ros_time = rospy.Time.from_sec(vrep_time/1000.)
            self.clock_pub.publish(Clock(clock=ros_time))

if __name__ == '__main__':
    rospy.init_node('vrep_clock_publisher')
    vrep_port = rospy.get_param('vrep/clock_port', 19997)
    VRepClockPublisher().run(vrep_port)
