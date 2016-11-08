from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def ros_to_list(joints):
    assert isinstance(joints, JointState)
    return joints.position


def list_to_ros(joints):
    return JointState(position=list(joints))


def state_to_jtp(joint_state):
    assert isinstance(joint_state, JointState)
    return JointTrajectoryPoint(positions=joint_state.position)