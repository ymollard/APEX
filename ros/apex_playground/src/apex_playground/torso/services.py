import rospy
from poppy_msgs.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class TorsoServices(object):
    def __init__(self, robot_name):
        self.set_compliant_srv = {}
        for group in ['left_arm', 'right_arm', 'full_robot']:
            self.set_compliant_srv[group] = {}
            name = '{}/{}/set_compliant'.format(robot_name, group)
            self.set_compliant_srv[group]['name'] = name
            rospy.loginfo("{} is waiting for service {}...".format(robot_name, name))
            rospy.wait_for_service(name)
            self.set_compliant_srv[group]['proxy'] = rospy.ServiceProxy(name, SetCompliant)

        self.idle_srv = {}
        self.demo_mode = rospy.get_param('demo_mode')
        for group in ['left_arm', 'right_arm', 'head']:
            self.idle_srv[group] = {}
            name = '{}/{}/set_idle_motion'.format(robot_name, group)
            self.idle_srv[group]['name'] = name
            if self.demo_mode:
                rospy.loginfo("{} is waiting for service {}...".format(robot_name, name))
                rospy.wait_for_service(name)
            self.idle_srv[group]['proxy'] = rospy.ServiceProxy(name, SetIdleMotion)

        self.execute_srv_name = '{}/execute'.format(robot_name)
        rospy.loginfo("{} is waiting for service {}...".format(robot_name, self.execute_srv_name))
        rospy.wait_for_service(self.execute_srv_name)
        self.execute_proxy = rospy.ServiceProxy(self.execute_srv_name, ExecuteTrajectory)
        self.reach_srv_name = '{}/reach'.format(robot_name)
        rospy.loginfo("{} is waiting for service {}...".format(robot_name, self.reach_srv_name))
        rospy.wait_for_service(self.reach_srv_name)
        self.reach_proxy = rospy.ServiceProxy(self.reach_srv_name, ReachTarget)
        self.torque_srv_name = '{}/set_torque_max'.format(robot_name)
        rospy.loginfo("{} is waiting for service {}...".format(robot_name, self.torque_srv_name))
        rospy.wait_for_service(self.torque_srv_name)
        self.set_torque_proxy = rospy.ServiceProxy(self.torque_srv_name, SetTorqueMax)
        rospy.loginfo("{} controllers are connected!".format(robot_name))

    def set_torque_max(self, value=100):
        request = SetTorqueMaxRequest()
        for m in ['l_shoulder_y', 'l_shoulder_x', 'l_arm_z', 'l_elbow_y']:
            request.joint_names.append(m)
            request.max_torques.append(value)
        return self.set_torque_proxy(request)

    def set_compliant(self, compliant):
        self.set_compliant_srv['full_robot']['proxy'](SetCompliantRequest(compliant=compliant))

    def set_arm_compliant(self, compliant, arm):
        if arm not in ['left', 'right']:
            raise ValueError("[TorsoServices.set_arm_compliant] Expecting arm left or right, got '{}".format(arm))
        self.set_compliant_srv['{}_arm'.format(arm)]['proxy'](SetCompliantRequest(compliant=compliant))

    def reach(self, positions, duration):
        self.reach_proxy(ReachTargetRequest(target=JointState(name=positions.keys(),
                                                              position=positions.values()),
                                            duration=rospy.Duration(duration)))

    def start_idle_motion(self, group='head'):
        if self.demo_mode:
            self.idle_srv[group]['proxy'](SetIdleMotionRequest(command=SetIdleMotionRequest.COMMAND_START))

    def stop_idle_motion(self, group='head'):
        if self.demo_mode:
            self.idle_srv[group]['proxy'](SetIdleMotionRequest(command=SetIdleMotionRequest.COMMAND_STOP))

    def execute(self, motion, duration):
        """
        Blocking function executing a motion
        :param motion: [{"motor_name": value_in_degrees}, {}...]
        :param duration: duration in secs
        """
        request = ExecuteTrajectoryRequest()
        request.trajectory.joint_names = motion.keys
        request.trajectory.time = rospy.Time.now()
        motion_length = len(duration)
        for point_id, point in enumerate(motion.values()):
            point = JointTrajectoryPoint(positions=point,
                                         time_from_start=rospy.Duration(float(point_id*duration)/motion_length))
            request.points.append(point)

        return self.execute_proxy(request)
