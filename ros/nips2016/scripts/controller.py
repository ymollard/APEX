#!/usr/bin/env python
import rospy
import json
from os.path import join
from rospkg import RosPack
from nips2016.controller import Perception, Learning, Torso, Ergo
from trajectory_msgs.msg import JointTrajectory


class Controller(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'general.json')) as f:
            self.params = json.load(f)
        self.torso = Torso()
        self.ergo = Ergo()
        self.learning = Learning()
        self.perception = Perception()
        rospy.loginfo('Controller fully started!')

    def reset(self):
        self.torso.reset()
        #self.ergo.reset()

    def run(self):
        while not rospy.is_shutdown():
            self.reset()
            if self.perception.help_pressed():
                recording = self.perception.record(human_demo=True, nb_points=self.params['nb_points'])
                self.learning.perceive(recording.torso_demonstration, recording.sensorial_demonstration)
            else:
                #space = self.perception.get_space_to_explore()
                #trajectory = self.learning.produce(space)
                trajectory = self.learning.produce().torso_trajectory
                self.torso.execute_trajectory(trajectory)  # TODO: blocking, non-blocking, action server?
                recording = self.perception.record(human_demo=False, nb_points=self.params['nb_points'])
                self.learning.perceive(JointTrajectory(), recording.sensorial_demonstration)  # TODO non-blocking
            # Many blocking calls: No sleep?

rospy.init_node("nips2016_controller")
Controller().run()
