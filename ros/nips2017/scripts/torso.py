#!/usr/bin/env python
import rospy
from nips2017.torso import Torso

rospy.init_node('torso')
Torso().run()
