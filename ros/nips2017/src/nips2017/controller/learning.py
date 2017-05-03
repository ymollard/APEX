import rospy
from nips2017.srv import *


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
        return call(PerceiveRequest(demo=demonstration))

    def produce(self, space_to_explore=0):
        call = self.services['produce']['call']
        return call(ProduceRequest(space_to_explore=space_to_explore))
