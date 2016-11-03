#!/usr/bin/env python
import rospy
from nips2016.perception import Perception

rospy.init_node('perception')
Perception().run()
rospy.spin()
