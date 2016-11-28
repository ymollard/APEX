import rospy
import json
import numpy as np
from os.path import join
from os import system
from rospkg.rospack import RosPack
from nips2016.srv import *
from nips2016.msg import SensorialState, Demonstration
from .aggregator import TopicAggregator
from ..tools import joints


class Perception(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'perception.json')) as f:
            self.params = json.load(f)
        self.rate = rospy.Rate(self.params['recording_rate'])

        # Serving these services
        self.service_name_get = "/nips2016/perception/get"
        self.service_name_record = "/nips2016/perception/record"
        # Using these services
        self.service_name_set_compliant = "/nips2016/torso/set_compliant"
        self.topics = TopicAggregator()  # All topics are read and stored in that object

    def run(self):
        for service in [self.service_name_set_compliant]:
            rospy.loginfo("Perception is waiting service {}".format(service))
            rospy.wait_for_service(service)
        self.set_torso_compliant_srv = rospy.ServiceProxy(self.service_name_set_compliant, SetTorsoCompliant)
        rospy.Service(self.service_name_get, GetSensorialState, self.cb_get)
        rospy.Service(self.service_name_record, Record, self.cb_record)
        rospy.loginfo("Done, perception is up!")

    def get(self):
        state = SensorialState(ball=self.topics.ball,
                               ergo=self.topics.ergo,
                               color=self.topics.light,
                               sound=self.topics.sound,
                               joystick_1=self.topics.joy1,
                               joystick_2=self.topics.joy2,
                               hand=self.topics.torso_l_eef)
        return state

    def wait_for_human_interaction(self, arm_threshold=1, joystick_threshold=0.15):
        rospy.loginfo("We are waiting for human interaction...")

        def detect_arm_variation():
            new_effort = np.array(self.topics.torso_l_j.effort)
            delta = np.absolute(effort - new_effort)
            return np.amax(delta) > arm_threshold

        def detect_joy_variation():
            return np.amax(np.abs(self.topics.joy1.axes)) > joystick_threshold

        effort = np.array(self.topics.torso_l_j.effort)
        rate = rospy.Rate(50)
        is_joystick_demo = None
        while not rospy.is_shutdown():
            if detect_arm_variation():
                is_joystick_demo = False
                break
            elif detect_joy_variation():
                is_joystick_demo = True
                break
            rate.sleep()
        return is_joystick_demo

    ################################# Service callbacks
    def cb_get(self, request):
        return GetSensorialStateResponse(state=self.get())

    def cb_record(self, request):
        response = RecordResponse()
        # TODO eventually keep trace of the last XX points to start recording prior to the start signal

        is_joystick_demo = False
        if request.human_demo.data:
            # Blocking... Wait for the user's grasp before recording...
            is_joystick_demo = self.wait_for_human_interaction()
            if not is_joystick_demo:
                self.set_torso_compliant_srv(SetTorsoCompliantRequest(compliant=True))

        rospy.loginfo("Recording {}...".format("a joystick demo" if is_joystick_demo else "an arm demo"))
        for point in range(request.nb_points.data):
            if rospy.is_shutdown():
                break
            if point % self.params["divider_nb_points_sensory"] == 0:
                response.demo.sensorial_demonstration.points.append(self.get())
            if not is_joystick_demo:
                response.demo.torso_demonstration.points.append(joints.state_to_jtp(self.topics.torso_l_j))
            self.rate.sleep()

        if not is_joystick_demo:
            self.set_torso_compliant_srv(SetTorsoCompliantRequest(compliant=False))

        if is_joystick_demo:
            response.demo.type_demo = Demonstration.TYPE_DEMO_JOYSTICK
        elif request.human_demo.data:
            response.demo.type_demo = Demonstration.TYPE_DEMO_ARM
        else:
            response.demo.type_demo = Demonstration.TYPE_DEMO_NORMAL


        rospy.loginfo("Recorded!")
        return response
