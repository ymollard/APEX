import json
from rospy import Subscriber
from rospkg import RosPack
from nips2016.msg import CircularState
from os.path import join
from numpy import arctan2, sqrt, pi
from geometry_msgs.msg import PoseStamped


class Environment(object):
    def __init__(self):
        self.end_effector = [42., 42., 42.]   # 0, 0, 0 is out of domain
        self.rospack = RosPack()
        Subscriber('/nips2016/ergo/end_effector_pose', PoseStamped, self._cb_eef)
        with open(join(self.rospack.get_path('nips2016'), 'config', 'environment.json')) as f:
            self.params = json.load(f)

    def _cb_eef(self, msg):
        self.end_effector = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def get_state(self):
        """
        Reduce the current joint configuration of the ergo in (angle, theta)
        :return: a CircularState
        """
        x, y, z = self.end_effector[0], self.end_effector[1], self.end_effector[2]
        theta = arctan2(y, x)
        elongation = sqrt(x*x + y*y)
        state = CircularState()
        state.extended = elongation > self.params['ergo']['threshold_extended']
        state.angle = theta
        return state

    def ball_to_color(self, x, y):
        """
        Reduce the given 2D ball position in color
        :param x:
        :param y:
        :return: hue value designating the color in [0, 255]
        """
        theta = arctan2(y, x)
        hue = int(theta*255/(2*pi))
        return hue
