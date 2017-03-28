#!/usr/bin/env python

from sensor_msgs.msg import Joy
from rospkg import RosPack
from os.path import join
import os
import sys
import rospy
import pygame
import json

os.environ["SDL_VIDEODRIVER"] = "dummy"
try:
    pygame.display.init()
except pygame.error:
    raise pygame.error("Can't connect to the console, from ssh enable -X forwarding")
pygame.joystick.init()


class HardwareJoystickPublisher(object):
    def __init__(self):
        self.joy_pub = rospy.Publisher('/nips2016/sensors/joystick/1', Joy, queue_size=1)
        self.joy_pub2 = rospy.Publisher('/nips2016/sensors/joystick/2', Joy, queue_size=1)

        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'ergo.json')) as f:
            self.params = json.load(f)

        self.rate = rospy.Rate(self.params['publish_rate'])

        if pygame.joystick.get_count() < 2:
            rospy.logerr("Sensors: Expecting 2 joysticks but found only {}, exiting".format(pygame.joystick.get_count()))
            sys.exit(0)
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick2 = pygame.joystick.Joystick(1)
            self.joystick.init()
            self.joystick2.init()

            if self.params['useless_joystick_id'] != int(self.joystick2.get_name()[-1]):
                useless_joy = self.joystick
                self.joystick = self.joystick2
                self.joystick2 = useless_joy

            rospy.loginfo('Initialized Joystick 1: {}'.format(self.joystick.get_name()))
            rospy.loginfo('Initialized Joystick 2: {}'.format(self.joystick2.get_name()))

    def publish_joy(self, x, y, publisher):
        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.axes.append(x)
        joy.axes.append(y)
        publisher.publish(joy)

    def run(self):
        while not rospy.is_shutdown():
            pygame.event.get()
            x = self.joystick.get_axis(0)
            y = self.joystick.get_axis(1)

            # Publishers
            self.publish_joy(x, y, self.joy_pub)
            x = self.joystick2.get_axis(0)
            y = self.joystick2.get_axis(1)
            self.publish_joy(x, y, self.joy_pub2)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hardware_joystick_publisher')
    HardwareJoystickPublisher().run()
