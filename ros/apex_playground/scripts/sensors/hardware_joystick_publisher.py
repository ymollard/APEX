#!/usr/bin/env python

import os
from inputs import devices

from threading import Thread
from sensor_msgs.msg import Joy
from rospkg import RosPack
from os.path import join
import sys
import rospy
import json

class JoystickThread(Thread):
    def __init__(self, pad):
        super(JoystickThread, self).__init__()
        self.setDaemon(True)
        self.pad = pad
        self.values = [0., 0.]

    def run(self):
        while not rospy.is_shutdown():
            events = self.pad.read()
            for event in events:
                try:
                    axis = ['ABS_X', 'ABS_Y'].index(event.code)
                except ValueError:
                    pass
                else:
                    self.values[axis] = (event.state-128)/128.  # Centering in [-1, 1]


class HardwareJoystickPublisher(object):
    def __init__(self):
        self.joy_pub = rospy.Publisher('sensors/joystick/1', Joy, queue_size=1)
        self.joy_pub2 = rospy.Publisher('sensors/joystick/2', Joy, queue_size=1)

        self.rospack = RosPack()

        self.rate = rospy.Rate(20)

        count = len(devices.gamepads)

        if count < 2:
            rospy.logerr("Sensors: Expecting 2 joysticks but found only {}, exiting".format(count))
            sys.exit(-1)
        
        gamepads = sorted(devices.gamepads, key=lambda pad: int(str(pad.name)[-1]))
        rospy.loginfo(gamepads)
        self.joysticks = [JoystickThread(pad) for pad in gamepads]
        [joystick.start() for joystick in self.joysticks]

    def publish_joy(self, x, y, publisher):
        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.axes.append(y)
        joy.axes.append(x)
        publisher.publish(joy)

    def run(self):
        while not rospy.is_shutdown():
            # Publishers
            self.publish_joy(self.joysticks[0].values[0], self.joysticks[0].values[1], self.joy_pub)
            self.publish_joy(self.joysticks[1].values[0], self.joysticks[1].values[1], self.joy_pub2)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hardware_joystick_publisher')
    HardwareJoystickPublisher().run()
