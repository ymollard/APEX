from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import rospy


def ros_to_list(joints):
    assert isinstance(joints, JointState)
    return joints.position


def list_to_ros(joints):
    return JointState(position=list(joints))


def state_to_jtp(joint_state):
    assert isinstance(joint_state, JointState)
    return JointTrajectoryPoint(positions=joint_state.position)


def wait_for_effort_variation(motors, threshold=1, rate=10):
    def get_effort():
        return [m.present_load for m in motors]

    def detect_variation():
        new_effort = np.array(get_effort())
        delta = np.absolute(effort - new_effort)
        return np.amax(delta) > threshold

    effort = np.array(get_effort())
    rate = rospy.Rate(rate)
    while not detect_variation() and not rospy.is_shutdown():
        rate.sleep()
