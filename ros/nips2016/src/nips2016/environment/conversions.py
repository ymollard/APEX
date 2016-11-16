import json
from rospkg import RosPack
from nips2016.msg import CircularState
from os.path import join
from numpy import arctan2, sqrt, pi


class EnvironmentConversions(object):
    def __init__(self):
        self.rospack = RosPack()
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

    @staticmethod
    def ball_to_color(state):
        """
        Reduce the given 2D ball position in color
        :param state: the requested circular state of the ball
        :return: hue value designating the color in [0, 255]
        """
        hue = min(255, max(0, int((state.angle + pi)*255/(2*pi))))
        return hue
