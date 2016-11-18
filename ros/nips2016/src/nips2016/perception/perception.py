import rospy
import json
from os.path import join
from os import system
from rospkg.rospack import RosPack
from nips2016.srv import *
from nips2016.msg import SensorialState
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
        self.service_name_setup_torso_rec = "/nips2016/torso/setup_recording"
        self.topics = TopicAggregator()  # All topics are read and stored in that object

    def run(self):
        for service in [self.service_name_setup_torso_rec]:
            rospy.loginfo("Perception is waiting service {}".format(service))
            rospy.wait_for_service(service)
        self.setup_torso_recording = rospy.ServiceProxy(self.service_name_setup_torso_rec, SetupTorsoRecording)
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

    ################################# Service callbacks
    def cb_get(self, request):
        return GetSensorialStateResponse(state=self.get())

    def cb_record(self, request):
        response = RecordResponse()
        # TODO eventually keep trace of the last XX points to start recording prior to the start signal
        if request.human_demo:
            # Blocking... Wait for the user's grasp before recording...
            self.setup_torso_recording(SetupTorsoRecordingRequest(wait_for_grasp=1))
        system('beep')
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - t0 < request.duration.data:
            response.sensorial_demonstration.points.append(self.get())
            response.torso_demonstration.points.append(joints.state_to_jtp(self.topics.torso_l_j))
            self.rate.sleep()
        system('beep')
        return response
