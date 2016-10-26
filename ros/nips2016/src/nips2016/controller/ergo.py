import rospy
from nips2016.srv import *


class Ergo(object):
    def __init__(self):
        self.services = {'exec_ergo': {'name': '/nips2016/ergo/execute', 'type': ExecuteErgoTrajectory},
                         'reset_ergo': {'name': '/nips2016/ergo/reset', 'type': Reset}}

        for service_name, service in self.services.items():
            rospy.loginfo("Waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def execute_trajectory(self, trajectory):
        call = self.services['exec_ergo']['call']
        return call(ExecuteErgoTrajectoryRequest(trajectory=trajectory))

    def reset(self):
        call = self.services['reset_ergo']['call']
        return call(ResetRequest())