#!/usr/bin/env python

import rospy
import os
import json
import datetime
from os.path import join
from rospkg.rospack import RosPack
from nips2016.srv import *
from nips2016.msg import Interests, Demonstration
from nips2016.learning import EnvironmentTranslator, Learning
from std_msgs.msg import String, Bool, UInt32, Float32
from threading import RLock


class LearningNode(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'learning.json')) as f:
            self.params = json.load(f)

        self.translator = EnvironmentTranslator()
        self.learning = Learning(self.translator.config, 
                                 n_motor_babbling=self.params["n_motor_babbling"], 
                                 explo_noise=self.params["explo_noise"], 
                                 choice_eps=self.params["choice_eps"], 
                                 enable_hand=self.params["enable_hand"])
        self.experiment_name = rospy.get_param("/nips2016/experiment_name", "experiment")
        self.source_name = rospy.get_param("/nips2016/source_name", "experiment")

        rospy.loginfo("Learning node will write {}".format(self.experiment_name))
        rospy.loginfo("Learning node will read {}".format(self.source_name))

        # User control
        self.lock_iteration = RLock()
        self.next_iteration = True
        self.ready_for_interaction = True
        self.focus = None
        self.set_iteration = -1
        self.demonstrate = None

        # Saved experiment files
        self.dir = join(self.rospack.get_path('nips2016'), 'logs')
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)
        self.stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.experiment_file = join(self.dir, self.stamp + '_' + self.experiment_name + '.pickle')
        self.source_file = join(self.dir, self.source_name + '.pickle')

        if self.source_name == "none":
            self.learning.start()
        else:
            self.learning.restart_from_end_of_file(self.source_file)

        # Serving these services
        self.service_name_perceive = "/nips2016/learning/perceive"
        self.service_name_produce = "/nips2016/learning/produce"
        self.service_name_set_interest = "/nips2016/learning/set_interest"
        self.service_name_set_iteration = "/nips2016/learning/set_iteration"
        self.service_name_demonstrate = "/nips2016/learning/demonstrate"

        # Publishing these topics
        self.pub_interests = rospy.Publisher('/nips2016/learning/interests', Interests, queue_size=1, latch=True)
        self.pub_focus = rospy.Publisher('/nips2016/learning/current_focus', String, queue_size=1, latch=True)
        self.pub_ready = rospy.Publisher('/nips2016/learning/ready_for_interaction', Bool, queue_size=1, latch=True)
        self.pub_iteration = rospy.Publisher('/nips2016/iteration', UInt32, queue_size=1, latch=True)

        # Using these services
        self.service_name_get_perception = "/nips2016/perception/get"
        for service in [self.service_name_get_perception]:
            rospy.loginfo("Learning  node is waiting service {}...".format(service))
            rospy.wait_for_service(service)
        self.get_state = rospy.ServiceProxy(self.service_name_get_perception, GetSensorialState)

    def run(self):
        rospy.Service(self.service_name_perceive, Perceive, self.cb_perceive)
        rospy.Service(self.service_name_produce, Produce, self.cb_produce)
        rospy.Service(self.service_name_set_interest, SetFocus, self.cb_set_focus)
        rospy.Service(self.service_name_set_iteration, SetIteration, self.cb_set_iteration)
        rospy.Service(self.service_name_demonstrate, Demonstrate, self.cb_demonstrate)
        rospy.loginfo("Learning is up!")

        rate = rospy.Rate(self.params['publish_rate'])
        try:
            while not rospy.is_shutdown():
                publish = False
                with self.lock_iteration:
                    if self.next_iteration:
                        publish = True
                        self.next_iteration = False
                if publish:
                    self.publish()
                self.pub_ready.publish(Bool(data=self.ready_for_interaction))
                rate.sleep()
        finally:
            rospy.loginfo("Saving file before exit into {}".format(self.experiment_file))
            self.learning.save(self.experiment_file)

    def publish(self):
        interests_array = self.learning.get_normalized_interests_evolution()
        interests = Interests()
        interests.names = self.learning.get_space_names()
        interests.num_iterations = UInt32(len(interests_array))
        interests.interests = [Float32(val) for val in interests_array.flatten()]

        self.pub_interests.publish(interests)
        self.pub_focus.publish(String(data=self.learning.get_last_focus()))
        self.pub_iteration.publish(UInt32(data=self.learning.get_iterations()))


    ################################# Service callbacks
    def cb_set_iteration(self, request):
        if self.ready_for_interaction:
            self.set_iteration = request.iteration.data
            self.ready_for_interaction = False
            self.learning.save(self.experiment_file)
            rospy.loginfo("Saving file before time travel into {}".format(self.experiment_file))
            self.stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.experiment_file = join(self.dir, self.stamp + '_set_iteration_' + self.experiment_name + '.pickle')
        return SetIterationResponse()

    def cb_set_focus(self, request):
        if self.ready_for_interaction:
            self.focus = request.space
            self.ready_for_interaction = False
        return SetFocusResponse()

    def cb_demonstrate(self, request):
        with self.lock_iteration:
            self.demonstrate = request.goal
        return DemonstrateResponse()

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

        # Regularly overwrite the results
        if self.learning.get_iterations() % self.params['save_every'] == 0:
            self.learning.save(self.experiment_file)
            rospy.loginfo("Saving file (periodic save) into {}".format(self.experiment_file))

        # This turn is over, check if we have a time travel pending...
        with self.lock_iteration:
            self.ready_for_interaction = True
            if self.set_iteration > -1:
                rospy.logwarn("Applying time travel to iteration {}".format(self.set_iteration))
                self.learning.restart_from_file(self.source_file, self.set_iteration)
                self.set_iteration = -1
        return PerceiveResponse()

    def cb_produce(self, request):
        rospy.loginfo("Learning node is requesting the current state")
        state = self.get_state(GetSensorialStateRequest()).state

        with self.lock_iteration:
            if self.demonstrate is None:
                rospy.loginfo("Learning node is producing...")
                w = self.learning.produce(self.translator.get_context(state), self.focus)
                self.focus = None
            else:
                rospy.loginfo("Learning node is demonstrating its abilities {}...".format(self.demonstrate))
                context = self.translator.get_context(state)
                w = self.learning.produce(context, goal=self.demonstrate)
                self.demonstrate = None

        trajectory_matrix = self.translator.w_to_trajectory(w)
        trajectory_msg = self.translator.matrix_to_trajectory_msg(trajectory_matrix)

        with self.lock_iteration:
            self.next_iteration = True

        response = ProduceResponse(torso_trajectory=trajectory_msg)
        return response

if __name__ == '__main__':
    rospy.init_node('learning')
    LearningNode().run()

