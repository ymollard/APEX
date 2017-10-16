import rospy
from apex_playground.srv import *


class Learning(object):
    def __init__(self):
        self.services = {'produce': {'name': 'learning/produce', 'type': Produce},
                         'perceive': {'name': 'learning/perceive', 'type': Perceive}}

        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def perceive(self, demonstration):
        call = self.services['perceive']['call']
        return call(PerceiveRequest(demo=demonstration)).success

    def produce(self, skill_to_demonstrate=""):
        call = self.services['produce']['call']
        return call(ProduceRequest(skill_to_demonstrate=skill_to_demonstrate))
