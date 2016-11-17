#!/usr/bin/env python
import rospy
from nips2016.torso import Torso

rospy.init_node('torso')
Torso().run()
