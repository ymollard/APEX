from nips2016.srv import *
from sensor_msgs.msg import Joy
import rospy


class Perception(object):
    def __init__(self):
        self.services = {'get_sensorial_state': {'name': '/nips2016/perception/get', 'type': GetSensorialState},
                         'record': {'name': '/nips2016/perception/record', 'type': Record}}
        self._help_msg = None

        for service_name, service in self.services.items():
            rospy.loginfo("Waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

        rospy.Subscriber('/nips2016/perception/help', Joy, self._cb_help_pressed)

    def _cb_help_pressed(self, msg):
        self._help_msg = msg

    def get_sensorial_state(self):
        call = self.services['get_sensorial_state']['call']
        return call(GetSensorialStateRequest())

    def record(self, duration=10.0):
        call = self.services['record']['call']
        return call(RecordRequest(duration=rospy.Duration(duration)))

    def help_pressed(self):
        pressed = self._help_msg is not None
        self._help_msg = None
        return pressed

    def get_space_to_explore(self):
        return rospy.get_param('/nips2016/perception/state_to_explore')