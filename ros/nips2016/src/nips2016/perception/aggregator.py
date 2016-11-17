from nips2016.msg import CircularState
from std_msgs.msg import UInt8, Float32, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Joy
import rospy


class TopicAggregator(object):
    def __init__(self):
        self.topics = {
            "ball": {"topic": "/nips2016/environment/ball", "sub": None, "data": CircularState(), "type": CircularState},
            "light": {"topic": "/nips2016/environment/light", "sub": None, "data": UInt8(), "type": UInt8},
            "sound": {"topic": "/nips2016/environment/sound", "sub": None, "data": Float32(), "type": Float32},
            "ergo_state": {"topic": "/nips2016/ergo/state", "sub": None, "data": CircularState(), "type": CircularState},
            "joy1": {"topic": "/nips2016/ergo/joysticks/1", "sub": None, "data": Joy(), "type": Joy},
            "joy2": {"topic": "/nips2016/ergo/joysticks/2", "sub": None, "data": Joy(), "type": Joy},
            "torso_l_j": {"topic": "/nips2016/torso/left_arm/joints", "sub": None, "data": JointState(), "type": JointState},
            "torso_l_eef": {"topic": "/nips2016/torso/left_arm/end_effector_pose", "sub": None, "data": PoseStamped(), "type": PoseStamped},
            "torso_r_eef": {"topic": "/nips2016/torso/right_arm/end_effector_pose", "sub": None, "data": PoseStamped(), "type": PoseStamped}
        }

        self.topics["ball"]["sub"] = rospy.Subscriber(self.topics["ball"]["topic"], self.topics["ball"]["type"], self.cb_ball)
        self.topics["light"]["sub"] = rospy.Subscriber(self.topics["light"]["topic"], self.topics["light"]["type"], self.cb_light)
        self.topics["sound"]["sub"] = rospy.Subscriber(self.topics["sound"]["topic"], self.topics["sound"]["type"], self.cb_sound)
        self.topics["ergo_state"]["sub"] = rospy.Subscriber(self.topics["ergo_state"]["topic"], self.topics["ergo_state"]["type"], self.cb_ergo)
        self.topics["joy1"]["sub"] = rospy.Subscriber(self.topics["joy1"]["topic"], self.topics["joy1"]["type"], self.cb_joy1)
        self.topics["joy2"]["sub"] = rospy.Subscriber(self.topics["joy2"]["topic"], self.topics["joy2"]["type"], self.cb_joy2)
        self.topics["torso_l_j"]["sub"] = rospy.Subscriber(self.topics["torso_l_j"]["topic"], self.topics["torso_l_j"]["type"], self.cb_torso_l_j)
        self.topics["torso_l_eef"]["sub"] = rospy.Subscriber(self.topics["torso_l_eef"]["topic"], self.topics["torso_l_eef"]["type"], self.cb_torso_l_eef)
        self.topics["torso_r_eef"]["sub"] = rospy.Subscriber(self.topics["torso_r_eef"]["topic"], self.topics["torso_r_eef"]["type"], self.cb_torso_r_eef)

    def cb_ball(self, msg):
        self.topics["ball"]["data"] = msg

    def cb_light(self, msg):
        self.topics["light"]["data"] = msg

    def cb_sound(self, msg):
        self.topics["sound"]["data"] = msg

    def cb_ergo(self, msg):
        self.topics["ergo_state"]["data"] = msg

    def cb_joy1(self, msg):
        self.topics["joy1"]["data"] = msg

    def cb_joy2(self, msg):
        self.topics["joy2"]["data"] = msg

    def cb_torso_l_j(self, msg):
        self.topics["torso_l_j"]["data"] = msg

    def cb_torso_l_eef(self, msg):
        self.topics["torso_l_eef"]["data"] = msg

    def cb_torso_r_eef(self, msg):
        self.topics["torso_r_eef"]["data"] = msg

    @property
    def ball(self):
        return self.topics["ball"]["data"]

    @property
    def light(self):
        return self.topics["light"]["data"]

    @property
    def sound(self):
        return self.topics["sound"]["data"]

    @property
    def ergo(self):
        return self.topics["ergo_state"]["data"]

    @property
    def joy1(self):
        return self.topics["joy1"]["data"]

    @property
    def joy2(self):
        return self.topics["joy2"]["data"]

    @property
    def torso_l_j(self):
        return self.topics["torso_l_j"]["data"]

    @property
    def torso_l_eef(self):
        return self.topics["torso_l_eef"]["data"]

    @property
    def torso_r_eef(self):
        return self.topics["torso_r_eef"]["data"]