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

        # Buttons switches + LEDs
        self.prefix = 'sensors'
        self.button_leds_topics = ['button_leds/help', 'button_leds/pause']
        self.buttons_topics = ['buttons/help', 'buttons/pause']
        self.subscribers = [rospy.Subscriber('/'.join([self.prefix, topic]), Bool, lambda msg: self._cb_button_pressed(msg, topic)) for topic in self.buttons_topics]
        self.publishers = {topic: rospy.Publisher('/'.join([self.prefix, topic]), Bool) for topic in self.button_leds_topics}
        self.button_leds_status = {topic: False for topic in self.button_leds_topics}
        self.button_pressed = {topic: False for topic in self.buttons_topics}
        self.last_press = {topic: rospy.Time(0) for topic in self.buttons_topics}

        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'perception.json')) as f:
            self.params = json.load(f)

    def _cb_button_pressed(self, msg, topic):
        if msg.data and rospy.Time.now() - self.last_press[topic] > rospy.Duration(self.params['duration_between_presses']):
            rospy.logwarn("Button {} pressed...".format(topic))
            self.button_pressed[topic] = True
            self.last_press[topic] = rospy.Time.now()

    def has_been_pressed(self, topic):
        if topic not in self.buttons_topics:
            return False
        pressed = self.button_pressed[topic]
        self.button_pressed[topic] = False
        return pressed

    def switch_led(self, topic, value=None):
        if value is None:
            self.button_leds_status[topic] = not self.button_leds_status[topic]
        elif value in [True, False]:
            self.button_leds_status[topic] = value
        self.publishers[topic].publish(Bool(data=self.button_leds_status[topic]))

    def record(self, human_demo, nb_points):
        call = self.services['record']['call']
        return call(RecordRequest(human_demo=Bool(data=human_demo), nb_points=UInt8(data=nb_points)))
