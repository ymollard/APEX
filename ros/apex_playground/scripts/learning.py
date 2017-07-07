#!/usr/bin/env python

import rospy
import os
import json
import datetime
from os.path import join
from rospkg.rospack import RosPack
from apex_playground.srv import *
from apex_playground.msg import Demonstration
from apex_playground.learning import EnvironmentTranslator, Learning
from std_msgs.msg import String, Bool, UInt32, Float32
from threading import RLock
from copy import copy
from os.path import isfile


class LearningNode(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'learning.json')) as f:
            self.params = json.load(f)

        self.translator = EnvironmentTranslator()
        self.learning = None

        # User control and critical resources
        self.lock_iteration = RLock()
        self.ready_for_interaction = True
        self.focus = None
        self.set_iteration = -1
        self.demonstrate = None

        # Saved experiment files
        self.dir = join(self.rospack.get_path('apex_playground'), 'logs')
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)
        # self.stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        # self.source_file = join(self.dir, self.source_name + '.pickle')
        self.main_experiment = True
        self.experiment_name = rospy.get_param('/experiment/name', 'apex')
        self.condition = ""
        self.trial = ""
        self.experiment_file = "/dev/null"

        # Serving these services
        self.service_name_perceive = "learning/perceive"
        self.service_name_produce = "learning/produce"
        self.service_name_set_interest = "learning/set_interest"
        self.service_name_set_iteration = "learning/set_iteration"
        self.service_name_demonstrate = "learning/assess"
        self.service_name_interests = "learning/get_interests"

        # Publishing these topics
        self.pub_focus = rospy.Publisher('learning/current_focus', String, queue_size=1, latch=True)
        self.pub_user_focus = rospy.Publisher('learning/user_focus', String, queue_size=1, latch=True)
        self.pub_ready = rospy.Publisher('learning/ready_for_interaction', Bool, queue_size=1, latch=True)
        self.pub_iteration = rospy.Publisher('iteration', UInt32, queue_size=1, latch=True)

        # Using these services
        self.service_name_get_perception = "perception/get"
        for service in [self.service_name_get_perception]:
            rospy.loginfo("Learning  node is waiting service {}...".format(service))
            rospy.wait_for_service(service)
        self.get_state = rospy.ServiceProxy(self.service_name_get_perception, GetSensorialState)

    def update_learner(self):
        condition = rospy.get_param('experiment/current/method')
        trial = rospy.get_param('experiment/current/trial')

        if condition != self.condition or trial != self.trial:
            with self.lock_iteration:
                if self.trial != '' and self.condition != '':                
                    rospy.logwarn("Learner closes and saves condition {} trial {}...".format(self.condition, self.trial+1))
                    self.learning.save(self.experiment_name, self.trial)

                rospy.logwarn("Learner opens condition {} trial {}...".format(condition, trial+1))
                self.experiment_name = rospy.get_param('/experiment/name', 'apex')
                self.experiment_file = join(self.dir, self.experiment_name + '.pickle') # if self.source_name == "none" else self.source_file

                self.learning = Learning(self.translator.config,
                                         condition=condition,
                                         n_motor_babbling=self.params["n_motor_babbling"],
                                         explo_noise=self.params["explo_noise"],
                                         choice_eps=self.params["choice_eps"],
                                         enable_hand=self.params["enable_hand"],
                                         normalize_interests=self.params["normalize_interests"])

                if False: #isfile(self.experiment_file):    # TODO: Restore only when supervisor.forward is back!
                    rospy.loginfo("Learning node appends data to {}".format(self.experiment_file))
                    self.learning.restart_from_end_of_file(self.experiment_file)
                else:
                    rospy.loginfo("Learning node created {}".format(self.experiment_file))
                    self.learning.start()

            rospy.loginfo("Learner loaded with condition {}!".format(condition))
            self.condition = condition
            self.trial = trial

    def cb_get_interests(self, request):
        interests_array = self.learning.get_normalized_interests_evolution()
        interests = GetInterestsResponse()
        if self.learning is not None:
            interests.names = self.learning.get_space_names()
            interests.num_iterations = UInt32(len(interests_array))
            interests.interests = [Float32(val) for val in interests_array.flatten()]
        return interests

    def run(self):
        rospy.Service(self.service_name_perceive, Perceive, self.cb_perceive)
        rospy.Service(self.service_name_produce, Produce, self.cb_produce)
        rospy.Service(self.service_name_set_interest, SetFocus, self.cb_set_focus)
        rospy.Service(self.service_name_set_iteration, SetIteration, self.cb_set_iteration)
        rospy.Service(self.service_name_demonstrate, Assess, self.cb_assess)
        rospy.Service(self.service_name_interests, GetInterests, self.cb_get_interests)
        rospy.loginfo("Learning is up!")

        rate = rospy.Rate(self.params['publish_rate'])
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        if self.learning is not None:
            with self.lock_iteration:
                focus = copy(self.focus)
                ready = copy(self.ready_for_interaction)

            self.pub_ready.publish(Bool(data=ready))
            self.pub_user_focus.publish(String(data=focus if focus is not None else ""))
            self.pub_focus.publish(String(data=self.learning.get_last_focus()))
            self.pub_iteration.publish(UInt32(data=self.learning.get_iterations()))


    ################################# Service callbacks
    def cb_set_iteration(self, request):
        with self.lock_iteration:
            ready = copy(self.ready_for_interaction)
            self.ready_for_interaction = False
            self.set_iteration = request.iteration.data

        into_past = request.iteration.data < self.learning.get_iterations()
        if ready:
            if into_past:
                if self.main_experiment:
                    self.learning.save(self.experiment_name, self.trial)
                self.main_experiment = False
                rospy.loginfo("Saving file before time travel into {}".format(self.experiment_file))
                #self.stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                #self.experiment_file = join(self.dir, self.stamp + '_set_iteration_' + self.experiment_name + '.pickle')
            else:
                self.main_experiment = True
        return SetIterationResponse()

    def cb_set_focus(self, request):
        with self.lock_iteration:
            # space is "" or e.g. "s_hand" to toggle focus on/off
            self.focus = request.space if len(request.space) > 0 else None
            self.ready_for_interaction = False
        return SetFocusResponse()

    def cb_assess(self, request):
        with self.lock_iteration:
            self.demonstrate = request.goal
            self.ready_for_interaction = False
        return AssessResponse()

    def cb_perceive(self, request):
        s = self.translator.sensory_trajectory_msg_to_list(request.demo.sensorial_demonstration)
        if request.demo.type_demo == Demonstration.TYPE_DEMO_ARM:
            torso_traj = self.translator.trajectory_msg_to_matrix(request.demo.torso_demonstration)
            torso_traj_w = self.translator.trajectory_to_w(torso_traj)
            rospy.loginfo("Learning node is perceiving sensory + torso trajectories for an arm demo")
            success = self.learning.perceive(s, m_demo=torso_traj_w)
        elif request.demo.type_demo == Demonstration.TYPE_DEMO_JOYSTICK:

            rospy.loginfo("Learning node is perceiving sensory + torso trajectories for a joystick demo")
            success = self.learning.perceive(s, j_demo=True)
        else:
            rospy.loginfo("Learning node is perceiving sensory trajectory only for a normal demo")
            success = self.learning.perceive(s)

        if not success:
            rospy.logerr("Learner could not perceive this trajectory")

        # This turn is over, check if we have a time travel pending...
        with self.lock_iteration:
            set_iteration = copy(self.set_iteration)
            self.set_iteration = -1

        if set_iteration > -1:
            rospy.logwarn("Applying time travel to iteration {}".format(set_iteration))
            self.learning.restart_from_file(self.experiment_file, set_iteration)

        # And savethe current iteration
        self.learning.save(self.experiment_name, self.trial)

        return PerceiveResponse()

    def cb_produce(self, request):
        with self.lock_iteration:
            # Check if we need a new learner
            self.update_learner()

            focus = copy(self.focus)
            demonstrate = copy(self.demonstrate)
            self.demonstrate = None

        rospy.loginfo("Learning node is requesting the current state")
        state = self.get_state(GetSensorialStateRequest()).state

        if demonstrate is None:
            rospy.loginfo("Learning node is producing...")
            w = self.learning.produce(self.translator.get_context(state), focus)
        else:
            rospy.logwarn("Assessing {}...".format(demonstrate))
            context = self.translator.get_context(state)
            w = self.learning.produce(context, goal=demonstrate)

        trajectory_matrix = self.translator.w_to_trajectory(w)
        trajectory_msg = self.translator.matrix_to_trajectory_msg(trajectory_matrix)

        self.ready_for_interaction = True

        response = ProduceResponse(torso_trajectory=trajectory_msg)
        return response

if __name__ == '__main__':
    rospy.init_node('learning')
    LearningNode().run()

