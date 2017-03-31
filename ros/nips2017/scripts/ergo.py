#!/usr/bin/env python

import rospy
from nips2017.ergo import Ergo

rospy.init_node('ergo')
Ergo().run()
