#!/usr/bin/env python

from nips2016.environment import BallTracking, EnvironmentConversions
from std_msgs.msg import Float32, UInt8
from nips2016.msg import CircularState
from rospkg import RosPack
from os.path import join
import json
import rospy


class Environment(object):
    def __init__(self):
        # Load parameters and hack the tuple conversions so that OpenCV is happy
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'environment.json')) as f:
            self.params = json.load(f)
        self.params['tracking']['ball']['lower'] = tuple(self.params['tracking']['ball']['lower'])
        self.params['tracking']['ball']['upper'] = tuple(self.params['tracking']['ball']['upper'])
        self.params['tracking']['arena']['lower'] = tuple(self.params['tracking']['arena']['lower'])
        self.params['tracking']['arena']['upper'] = tuple(self.params['tracking']['arena']['upper'])

        self.tracking = BallTracking(self.params)
        self.conversions = EnvironmentConversions()
        self.ball_pub = rospy.Publisher('/nips2016/environment/ball', CircularState, queue_size=1)
        self.light_pub = rospy.Publisher('/nips2016/environment/light', UInt8, queue_size=1)
        self.sound_pub = rospy.Publisher('/nips2016/environment/sound', Float32, queue_size=1)
        self.rate = rospy.Rate(self.params['rate'])

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

            hsv, mask_ball, mask_arena = self.tracking.get_images(frame)
            ball_center, _ = self.tracking.find_center('ball', frame, mask_ball, 20)
            arena_center, arena_radius = self.tracking.find_center('arena', frame, mask_arena, 100)

            if ball_center is not None:
                x_ball, y_ball = ball_center
                circular_state = self.conversions.get_state(x_ball, y_ball)
                self.update_light(circular_state)
                self.update_sound(circular_state)
                self.ball_pub.publish(circular_state)

            if debug:
                self.tracking.draw_images(frame, hsv, mask_ball, mask_arena, arena_center,
                                          None if arena_radius is None else int(arena_radius/self.params['tracking']['ring_divider']))
            self.rate.sleep()



if __name__ == '__main__':
    rospy.init_node('environment')
    Environment().run()