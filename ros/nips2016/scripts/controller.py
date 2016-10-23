#!/usr/bin/env python
import rospy
from nips2016.controller import Perception, Learning, Torso, Ergo


class Controller(object):
    def __init__(self, frequency=2, experiment_duration=10.0):
        self.interaction_loop_rate = rospy.Rate(frequency)
        self.experiment_duration = experiment_duration
        self.torso = Torso()
        self.ergo = Ergo()
        self.learning = Learning()
        self.perception = Perception()
        rospy.loginfo('Controller fully started!')

    def reset(self):
        self.torso.reset()
        self.ergo.reset()

    def run(self):
        while not rospy.is_shutdown():
            self.reset()
            if self.perception.help_pressed():
                self.torso.set_compliant(True)
            else:
                space = self.perception.get_space_to_explore()
                trajectory = self.learning.produce(space)
                self.torso.execute_trajectory(trajectory)  # TODO: blocking, non-blocking, action server?
            recording = self.perception.record(duraction=self.experiment_duration)
            self.learning.perceive(recording.torso_demonstration, recording.sensorial_demonstration)  # TODO non-blocking
            self.interaction_loop_rate.sleep()

rospy.init_node("nips2016_controller")
Controller().run()
