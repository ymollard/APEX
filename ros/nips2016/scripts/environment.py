#!/usr/bin/env python

from nips2016.environment import BallTracking, EnvironmentConversions
from std_msgs.msg import Float32, UInt8
from nips2016.msg import CircularState
import rospy


class Environment(object):
    def __init__(self):
        self.tracking = BallTracking()
        self.conversions = EnvironmentConversions()
        self.ball_pub = rospy.Publisher('/nips2016/environment/ball', CircularState, queue_size=1)
        self.light_pub = rospy.Publisher('/nips2016/environment/light', UInt8, queue_size=1)
        self.sound_pub = rospy.Publisher('/nips2016/environment/sound', Float32, queue_size=1)

    def update_light(self, state):
        self.light_pub.publish(UInt8(data=self.conversions.ball_to_color(state)))

    def update_sound(self, state):
        self.sound_pub.publish(state.angle if state.extended else 0.)  # TODO rescale

    def run(self):
        if not self.tracking.open():
            rospy.logerr("Cannot open the webcam, exiting")
            return

        while not rospy.is_shutdown():
            debug = rospy.get_param('/nips2016/perception/debug', False)
            grabbed, frame = self.tracking.read()

            if not grabbed:
                rospy.logerr("Cannot grab image from webcam, exiting")
                break

            hsv, mask = self.tracking.get_images(frame)
            ball_center = self.tracking.find_center(frame, hsv, mask)

            if ball_center is not None:
                x_ball, y_ball = ball_center
                circular_state = self.conversions.get_state(x_ball, y_ball)
                self.update_light(circular_state)
                self.update_sound(circular_state)
                self.ball_pub.publish(circular_state)

            if debug:
                self.tracking.draw_images(frame, hsv, mask)

if __name__ == '__main__':
    rospy.init_node('environment')
    Environment().run()