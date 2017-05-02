#!/usr/bin/env python

import pypot.vrep.remoteApiBindings.vrep as vrep # Pypot has the path to V-Rep bindings, but that could also be a direct import
import rospy
import time
from rosgraph_msgs.msg import Clock


class VRepClockPublisher(object):
    """
    This is a loop publishing VRep time to ROS at 100Hz
    It owns the the main VRep Remote API port and opens the other ports for other nodes
    """
    def __init__(self, rate_hz=10):
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        self.rate = rate_hz
        vrep_port = rospy.get_param('/vrep_port', 19997)
        self.simulation_id = -1

    def run(self, initial_vrep_port):
        self.simulation_id = vrep.simxStart('127.0.0.1', initial_vrep_port, True, False, 5000, 5)
        self.open_ports(initial_vrep_port)
        while not rospy.is_shutdown():
            vrep_time = vrep.simxGetLastCmdTime(self.simulation_id)
            if vrep_time > 0:
                ros_time = rospy.Time.from_sec(vrep_time/1000.)
                self.clock_pub.publish(Clock(clock=ros_time))
            time.sleep(1./self.rate)

if __name__ == '__main__':
    rospy.init_node('vrep_clock_publisher')
    initial_vrep_port = rospy.get_param('/vrep/initial_port', 19997)
    VRepClockPublisher().run(initial_vrep_port)
