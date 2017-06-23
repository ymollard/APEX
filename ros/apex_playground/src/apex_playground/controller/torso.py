import rospy
import json
from os.path import join
from poppy_msgs.srv import ExecuteTrajectory, ExecuteTrajectoryRequest, SetTorqueMax, SetTorqueMaxRequest
from apex_playground.srv import Reset, ResetRequest
from rospkg import RosPack


class Torso(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'torso.json')) as f:
            self.params = json.load(f)
        self.services = {'exec_torso': {'name': '{}/execute'.format(self.params['robot_name']), 'type': ExecuteTrajectory},
                         'reset_torso': {'name': 'torso/reset', 'type': Reset},
                         'set_torque_max': {'name': '{}/set_torque_max'.format(self.params['robot_name']), 'type': SetTorqueMax}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def reset(self, slow):
        call = self.services['reset_torso']['call']
        return call(ResetRequest(slow=slow))

    def set_torque_max(self, torque_max=100):
        call = self.services['set_torque_max']['call']
        joints = [
                  #'abs_z',
                  #'bust_y',
                  #'bust_x',
                  #'head_z',
                  #'head_y',
                  'l_shoulder_y',
                  'l_shoulder_x',
                  'l_arm_z',
                  'l_elbow_y',
                  'r_shoulder_y',
                  'r_shoulder_x',
                  'r_arm_z',
                  'r_elbow_y'
        ]
        return call(SetTorqueMaxRequest(joint_names=joints, max_torques=[torque_max]*len(joints)))

    def execute_trajectory(self, trajectory):
        call = self.services['exec_torso']['call']
        return call(ExecuteTrajectoryRequest(trajectory=trajectory))

