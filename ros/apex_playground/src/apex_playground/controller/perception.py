from std_msgs.msg import Bool, UInt8
from rospkg import RosPack
from os.path import join
from apex_playground.srv import RecordRequest, Record
import rospy
import json


class Perception(object):
    def __init__(self):
        self.services = {'record': {'name': 'perception/record', 'type': Record}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

        self.buttons_topics = ['ergo/buttons/help', 'ergo/buttons/pause']
        self.subscribers = [rospy.Subscriber(topic, Bool, lambda msg: self._cb_button_pressed(msg, topic)) for topic in self.buttons_topics]
        self.button_pressed = {topic: False for topic in self.buttons_topics}
        self.last_press = {topic: rospy.Time(0) for topic in self.buttons_topics}
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'perception.json')) as f:
            self.params = json.load(f)

    def _cb_button_pressed(self, msg, topic):
        if msg.data and rospy.Time.now() - self.last_press > rospy.Duration(self.params['duration_between_presses']):
            rospy.logwarn("Button {} pressed...".format(topic))
            self.button_pressed[topic] = True
            self.last_press[topic] = rospy.Time.now()

    def has_been_pressed(self, topic):
        if topic not in self.buttons_topics:
            return False
        pressed = self.button_pressed[topic]
        self.button_pressed[topic] = False
        return pressed

    def record(self, human_demo, nb_points):
        call = self.services['record']['call']
        return call(RecordRequest(human_demo=Bool(data=human_demo), nb_points=UInt8(data=nb_points)))
