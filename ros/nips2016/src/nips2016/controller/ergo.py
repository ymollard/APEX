import rospy
from nips2016.srv import *


class Ergo(object):
    def __init__(self):
        self.services = {'reset_ergo': {'name': '/nips2016/ergo/reset', 'type': Reset}}

        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def reset(self, slow=True):
        call = self.services['reset_ergo']['call']
        return call(ResetRequest(slow=slow))