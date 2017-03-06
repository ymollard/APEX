import rospy
import json
from os import system
from nips2017.srv import *
from threading import RLock
from rospkg import RosPack
from os.path import join
from .idle import UpperBodyIdleMotion, HeadIdleMotion
from .services import TorsoServices


class Torso(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2017'), 'config', 'torso.json')) as f:
            self.params = json.load(f)

        self.publish_rate = rospy.Rate(self.params['publish_rate'])

        self.primitive_head = None
        self.primitive_right = None

        # Protected resources
        self.in_rest_pose = False
        self.robot_lock = RLock()

        # Used services
        self.torso = TorsoServices(self.params['robot_name'])

        # Proposed servuces
        self.reset_srv_name = '/nips2017/torso/reset'
        self.reset_srv = None

    def go_to_start(self, duration=5):
        d = {"abs_z": 75,
             "bust_y": 0,
             "bust_x": 0,
             "head_z": 0,
             "head_y": -0,
             "l_shoulder_y": 0,
             "l_shoulder_x": 0,
             "l_arm_z": 20,
             "l_elbow_y": 0,
             "r_shoulder_y": 0,
             "r_shoulder_x": 0,
             "r_arm_z": 0,
             "r_elbow_y": 0}
        self.torso.set_compliant(False)
        self.torso.reach(d, duration)
        rospy.sleep(duration)

    def go_to_rest(self, slow):
        with self.robot_lock:
            duration = 4 if slow else 0.5
            #self.set_torque_limits(60)
            #self.torso.goto_position({'l_shoulder_y': 13, 'l_shoulder_x': 20, 'l_elbow_y': -25}, duration)
            #rospy.sleep(duration)
            self.torso.reach({'l_shoulder_y': -25, 'l_shoulder_x': 40, 'l_arm_z': 30, 'l_elbow_y': 0}, duration)
            rospy.sleep(duration)
            rospy.sleep(0.5)
            self.in_rest_pose = True
            #self.set_torque_limits()

    #def set_torque_limits(self, value=None):
    #    for m in self.torso.l_arm:
    #        m.torque_limit = self.params['torques'] if value is None else value

    def run(self):
        self.reset_srv = rospy.Service(self.reset_srv_name, Reset, self._cb_reset)
        self.go_to_start()
        #self.primitive_head = UpperBodyIdleMotion(self.torso, 15)
        #self.primitive_right = HeadIdleMotion(self.torso, 15)
        #self.primitive_head.start()
        #self.primitive_right.start()

        rospy.spin()
        #self.primitive_right.stop()
        #self.primitive_head.stop()

    def _cb_reset(self, request):
        rospy.loginfo("Resetting Torso{}...".format(" in slow mode" if request.slow else ""))

        if request.slow:
            with self.robot_lock:
                self.torso.set_arm_compliant(False, 'left')
                system('beep')
                rospy.sleep(1)
                self.go_to_rest(True)
        else:
            with self.robot_lock:
                self.torso.set_arm_compliant(False, 'left')
                self.go_to_rest(False)
        return ResetResponse()

