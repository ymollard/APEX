#!/usr/bin/env python
import rospy
import json
from os.path import join
from rospkg import RosPack
from nips2016.controller import Perception, Learning, Torso, Ergo
from trajectory_msgs.msg import JointTrajectory
from copy import deepcopy


class Controller(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'general.json')) as f:
            self.params = json.load(f)
        self.experiment = rospy.get_param('/nips2016/experiment')
        self.torso = Torso()
        self.ergo = Ergo()
        self.learning = Learning()
        self.perception = Perception()
        rospy.loginfo('Controller fully started!')

    def run(self):
        condition = self.experiment['sequence'][self.experiment['current']['condition']]
        start_trial = self.experiment['sequence'][self.experiment['current']['trial']]
        start_iteration = self.experiment['sequence'][self.experiment['current']['iteration']]
        while not rospy.is_shutdown() and self.experiment['current']['condition'] < len(self.experiment['sequence']):
            params = self.experiment['params'][condition]
            for trial in range(start_trial, params[condition]['num_trials']):
                for iteration in range(start_iteration, params[condition]['num_iterations']):
                    try:
                        if rospy.is_shutdown():
                            return

                        if iteration % self.params['ergo_reset'] == 1:
                            self.ergo.reset(True)

                        self.execute_iteration(iteration, trial, params[condition])
                    finally:
                        updated_experiment = deepcopy(self.experiment)
                        updated_experiment['condition'] = condition
                        updated_experiment['trial'] = trial
                        updated_experiment['iteration'] = iteration
                        rospy.set_param('/nips2016/experiment', updated_experiment)

    def execute_iteration(self, iteration, trial, params):
        rospy.logwarn("#### Iteration {}/{} trial {}/{}".format(iteration, params['num_iterations'],
                                                                trial, params['num_trials']))
        if self.perception.help_pressed():
            rospy.sleep(1.5)  # Wait for the robot to fully stop
            recording = self.perception.record(human_demo=True, nb_points=self.params['nb_points'])
            self.torso.reset(slow=True)
        else:
            trajectory = self.learning.produce().torso_trajectory
            self.torso.execute_trajectory(trajectory)  # TODO: blocking, non-blocking, action server?
            recording = self.perception.record(human_demo=False, nb_points=self.params['nb_points'])
            recording.demo.torso_demonstration = JointTrajectory()
            self.torso.reset()
        self.learning.perceive(recording.demo)  # TODO non-blocking

rospy.init_node("nips2016_controller")
Controller().run()
