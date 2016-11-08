import cv2
import rospy
from nips2016.srv import GetSensorialState, GetSensorialStateResponse, Record, RecordResponse
from nips2016.msg import SensorialState, CircularState
from nips2016.tools import joints
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Joy
from .environment import Environment
from ..tools import joints


class Perception(object):
    def __init__(self):
        self.environment = Environment()
        self.rate = rospy.get_param('/nips2016/perception/rate')
        self.service_name_get = "/nips2016/perception/get"
        self.service_name_record = "/nips2016/perception/record"

        self.sub_ergo_joints = rospy.Subscriber('/nips2016/ergo/joints', JointState, self.cb_ergo_joints)
        self.sub_torso_joints = rospy.Subscriber('/nips2016/torso/joints', JointState, self.cb_torso_joints)
        self.sub_joy = rospy.Subscriber('/nips2016/torso/joystick', Joy, self.cb_torso_joy)
        self.ergo_joints = JointState(position=[0, 0, 0, 0, 0, 0])
        self.torso_joints = JointState()
        self.torso_joy = Joy()

    def run(self):
        rospy.Service(self.service_name_get, GetSensorialState, self.cb_get)
        rospy.Service(self.service_name_record, Record, self.cb_record)

    def _get_ball_position(self):
        """
        Return the current ball position
        :return: [e, theta]
        """
        # TODO openCV
        return False, 42

    def get(self):
        # TODO read the full sensorial state
        state = SensorialState()

        # Ball
        ball = self._get_ball_position()
        state.ball.extended = ball[0]
        state.ball.angle = ball[1]

        # Ergo joints
        state.ergo = self.environment.get_state()

        # LED colors
        state.color = self.environment.ball_to_color(*ball)

        # Sound
        # state.sound =

        # Joystick
        # state.joystick =

        # Hand
        # state.hand =

        return state

    ################################# Topic callbacks
    def cb_ergo_joints(self, joints):
        self.ergo_joints = joints

    def cb_torso_joints(self, joints):
        self.torso_joints = joints

    def cb_torso_joy(self, joy):
        self.torso_joy = joy

    ################################# Service callbacks
    def cb_get(self, request):
        return GetSensorialStateResponse(state=self.get())

    def cb_record(self, request):
        response = RecordResponse()
        rate = rospy.Rate(self.rate)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - t0 < request.duration:
            response.sensorial_demonstration.points.append(self.get())
            response.torso_demonstration.append(joints.state_to_jtp(self.torso_joints))
            rate.sleep()
        return response