import rospy
import json
from os.path import join
from poppy_msgs.srv import ExecuteTrajectory, ExecuteTrajectoryRequest
from nips2017.srv import Reset, ResetRequest
from rospkg import RosPack


class Torso(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2017'), 'config', 'torso.json')) as f:
            self.params = json.load(f)
        self.services = {'exec_torso': {'name': '/{}/execute'.format(self.params['robot_name']), 'type': ExecuteTrajectory},
                         'reset_torso': {'name': '/nips2017/torso/reset', 'type': Reset}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def reset(self, slow):
        call = self.services['reset_torso']['call']
        return call(ResetRequest(slow=slow))

    def execute_trajectory(self, trajectory):
        call = self.services['exec_torso']['call']
        return call(ExecuteTrajectoryRequest(trajectory=trajectory))

