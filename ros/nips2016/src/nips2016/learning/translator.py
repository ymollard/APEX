from .dmp.mydmp import MyDMP
from explauto.utils import bounds_min_max
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import rospy


class EnvironmentTranslator(object):
    """
    This class gives sense to all the numerical parameters used by the learning and handles the transformation:
    Huge list of floats <=> meaningful class instances

    Therefore it also stores the joint names/order
    """
    def __init__(self):
        # DMP PARAMETERS
        self.n_dmps = 4
        self.n_bfs = 7
        self.timesteps = 30
        self.max_params = np.array([300.] * self.n_bfs * self.n_dmps + [1.] * self.n_dmps)
        self.motor_dmp = MyDMP(n_dmps=self.n_dmps, n_bfs=self.n_bfs, timesteps=self.timesteps, max_params=self.max_params)
        self.context = {}
        self.config = dict(m_mins=[-1.]*32,
                           m_maxs=[1.]*32,
                           s_mins=[-1.]*132,
                           s_maxs=[1.]*132)

    def trajectory_to_w(self, m_traj):
        assert m_traj.shape == (self.timesteps, self.n_dmps)
        return self.motor_dmp.imitate(m_traj) / self.max_params

    def w_to_trajectory(self, w):
        return bounds_min_max(self.motor_dmp.trajectory(np.array(w) * self.max_params), self.n_dmps * [-1.], self.n_dmps * [1.])

    def get_context(self, state):
        return [state.ergo.angle, state.ball.angle]

    def sensory_trajectory_msg_to_list(self, state):
        def flatten(list2d):
            return [element2 for element1 in list2d for element2 in element1]

        state_dict = {}
        state_dict['hand'] = flatten([(point.hand.pose.position.x, point.hand.pose.position.y, point.hand.pose.position.z) for point in state.points])
        state_dict['joystick_1'] = flatten([point.joystick_1.axes for point in state.points])
        state_dict['joystick_2'] = flatten([point.joystick_2.axes for point in state.points])
        state_dict['ergo'] = flatten([(point.ergo.angle, float(point.ergo.extended)) for point in state.points])
        state_dict['ball'] = flatten([(point.ball.angle, float(point.ball.extended)) for point in state.points])
        state_dict['light'] = [point.color.data for point in state.points]
        state_dict['sound'] = [point.sound.data for point in state.points]

        self.context = {'ball': state_dict['ball'][0],
                        'ergo': state_dict['ergo'][0]}
        rospy.loginfo("Context {}".format(self.context))

        assert len(state_dict['hand']) == 30, len(state_dict['hand'])
        assert len(state_dict['joystick_1']) == 20, len(state_dict['joystick_1'])
        assert len(state_dict['joystick_2']) == 20, len(state_dict['joystick_2'])
        assert len(state_dict['ergo']) == 20, len(state_dict['ergo'])
        assert len(state_dict['ball']) == 20, len(state_dict['ball'])
        assert len(state_dict['light']) == 10, len(state_dict['light'])
        assert len(state_dict['sound']) == 10, len(state_dict['sound'])

        # Concatenate all these values in a huge 132-float list
        return [self.context['ergo'], self.context['ball']] + [value for space in ['hand', 'joystick_1', 'joystick_2', 'ergo', 'ball', 'light', 'sound'] for value in state_dict[space]]

    def matrix_to_trajectory_msg(self, matrix_traj):
        assert matrix_traj.shape == (self.timesteps, self.n_dmps)
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ['l_shoulder_y', 'l_shoulder_x', 'l_arm_z', 'l_elbow_y']
        traj.points = [JointTrajectoryPoint(positions=list(matrix_traj[point])) for point in range(len(matrix_traj))]
        return traj

    def trajectory_msg_to_matrix(self, trajectory):
        matrix = np.array([point.positions for point in trajectory.points])
        assert matrix.shape == (self.timesteps, self.n_dmps)
        return matrix
