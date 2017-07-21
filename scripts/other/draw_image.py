#!/usr/bin/env python

import rospy
import numpy
import cv2
import sys
from std_msgs.msg import Float32MultiArray

rospy.init_node('image_draw')


def callback(msg):
    frame = numpy.reshape(msg.data, [d.size for d in msg.layout.dim]).astype(numpy.uint8)
    cv2.imshow("Frame", frame)
    cv2.waitKey(1)

rospy.Subscriber(sys.argv[1], Float32MultiArray, callback)
rospy.spin()
