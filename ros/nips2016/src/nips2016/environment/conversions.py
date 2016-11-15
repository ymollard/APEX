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

    def get_state(self, x, y):
        """
        Reduce the current joint configuration of the ergo in (angle, theta)
        :return: a CircularState
        """
        theta = arctan2(y, x)
        elongation = sqrt(x*x + y*y)
        state = CircularState()
        state.extended = elongation > self.params['ergo']['threshold_extended']
        state.angle = theta
        return state

    @staticmethod
    def ball_to_color(state):
        """
        Reduce the given 2D ball position in color
        :param x:
        :param y:
        :return: hue value designating the color in [0, 255]
        """
        hue = min(255, max(0, int((state.angle - 3.14)*255/(2*pi))))
        return hue
