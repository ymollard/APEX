#!/usr/bin/env python
import rospy
from apex_playground.torso import Torso

rospy.init_node('torso')
Torso().run()
