#!/usr/bin/env python

from apex_playground.environment import BallTracking, EnvironmentConversions
from std_msgs.msg import Float32, UInt8
from apex_playground.msg import CircularState
from rospkg import RosPack
from os.path import join
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import json
import rospy


class Environment(object):
    def __init__(self):
        # Load parameters and hack the tuple conversions so that OpenCV is happy
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'environment.json')) as f:
            self.params = json.load(f)
        self.params['tracking']['ball']['lower'] = tuple(self.params['tracking']['ball']['lower'])
        self.params['tracking']['ball']['upper'] = tuple(self.params['tracking']['ball']['upper'])
        self.params['tracking']['arena']['lower'] = tuple(self.params['tracking']['arena']['lower'])
        self.params['tracking']['arena']['upper'] = tuple(self.params['tracking']['arena']['upper'])

        self.tracking = BallTracking(self.params)
        self.conversions = EnvironmentConversions()
        self.ball_pub = rospy.Publisher('environment/ball', CircularState, queue_size=1)
        self.light_pub = rospy.Publisher('environment/light', UInt8, queue_size=1)
        self.image_pub = rospy.Publisher('environment/image', Float32MultiArray, queue_size=1)
        self.sound_pub = rospy.Publisher('environment/sound', Float32, queue_size=1)
        self.rate = rospy.Rate(self.params['rate'])

    def update_light(self, state):
        self.light_pub.publish(UInt8(self.conversions.ball_to_color(state)))

    def update_sound(self, state):
        self.sound_pub.publish(Float32(self.conversions.ball_to_sound(state)))  # TODO rescale

    def run(self):
        if not self.tracking.open(*self.params['tracking']['resolution']):
            rospy.logerr("Cannot open the webcam, exiting")
            return

        while not rospy.is_shutdown():
            debug = rospy.get_param('environment/debug', False)
            grabbed, frame = self.tracking.read()

            if not grabbed:
                rospy.logerr("Cannot grab image from webcam, exiting")
                break

            hsv, mask_ball, mask_arena = self.tracking.get_images(frame)
            min_radius_ball = self.params['tracking']['resolution'][0]*self.params['tracking']['resolution'][1]/20000.
            ball_center, _ = self.tracking.find_center('ball', frame, mask_ball, min_radius_ball)
            min_radius_arena = self.params['tracking']['resolution'][0]*self.params['tracking']['resolution'][1]/2000.
            arena_center, arena_radius = self.tracking.find_center('arena', frame, mask_arena, min_radius_arena)
            ring_radius = int(arena_radius/self.params['tracking']['ring_divider']) if arena_radius is not None else None

            if ball_center is not None and arena_center is not None:
                circular_state = self.conversions.get_state(ball_center, arena_center, ring_radius)
                self.update_light(circular_state)
                self.update_sound(circular_state)
                self.ball_pub.publish(circular_state)

            if debug:
                frame = self.tracking.draw_images(frame, hsv, mask_ball, mask_arena, arena_center, ring_radius)
                image = Float32MultiArray()
                for dim in range(len(frame.shape)):
                    image.layout.dim.append(MultiArrayDimension(size=frame.shape[dim], label=str(frame.dtype)))
                length = reduce(int.__mul__, frame.shape)
                image.data = list(frame.reshape(length))
                self.image_pub.publish(image)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('environment')
    Environment().run()
