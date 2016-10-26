import rospy
from nips2016.srv import *


class Learning(object):
    def __init__(self):
        self.services = {'produce': {'name': '/nips2016/learner/produce', 'type': Produce},
                         'perceive': {'name': '/nips2016/learner/perceive', 'type': Perceive}}

        for service_name, service in self.services.items():
            rospy.loginfo("Waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def perceive(self, torso_demonstration, sensorial_demonstration):
        call = self.services['perceive']['call']
        return call(PerceiveRequest(torso_demonstration=torso_demonstration,
                                    sensorial_demonstration=sensorial_demonstration))

    def produce(self, space_to_explore=0):
        call = self.services['produce']['call']
        return call(ProduceRequest(space_to_explore=space_to_explore))
