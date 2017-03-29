import json
from rospkg import RosPack
from nips2016.msg import CircularState
from os.path import join
from numpy import arctan2, sqrt, pi


class EnvironmentConversions(object):
    def __init__(self):
        self.rospack = RosPack()
        self.last_angle = None
        with open(join(self.rospack.get_path('nips2016'), 'config', 'environment.json')) as f:
            self.params = json.load(f)

    def get_state(self, ball_center, arena_center, ring_radius):
        """
        Reduce the current joint configuration of the ergo in (angle, theta)
        :return: a CircularState
        """
        x, y = (ball_center[0] - arena_center[0], ball_center[1] - arena_center[1])
        elongation = sqrt(x*x + y*y)
        theta = arctan2(y, x)
        state = CircularState()
        state.extended = elongation > ring_radius
        state.angle = theta
        return state

    def ball_to_color(self, state):
        """
        Reduce the given 2D ball position in color
        :param state: the requested circular state of the ball
        :return: hue value designating the color in [0, 255]
        """
        max_speed = 0.2
        min_speed = 0.05
        if self.last_angle is None:
            hue = 0
        else:
            distance = abs(state.angle-self.last_angle)
            speed = max(min_speed, min(max_speed, min(distance, 2*pi-distance)))
            hue = int((speed - min_speed)/(max_speed - min_speed)*255)
        self.last_angle = state.angle
        return hue
