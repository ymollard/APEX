#!/usr/bin/env python

import rospy
import os
import json
from os.path import join
from rospkg.rospack import RosPack
from nips2016.srv import *
from nips2016.msg import Interests
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
                                 choice_eps=self.params["choice_eps"])
        self.learning.start()
        self.experiment_name = rospy.get_param("/nips2016/experiment_name", "experiment")
        self.lock_next_iteration = RLock()
        self.next_iteration = True
        self.ready_for_interaction = True
        self.focus = None

        # Serving these services
        self.service_name_perceive = "/nips2016/learning/perceive"
        self.service_name_produce = "/nips2016/learning/produce"
        self.service_name_set_interest = "/nips2016/learning/set_interest"
        self.service_name_set_iteration = "/nips2016/learning/set_iteration"

        # Publishing these topics
        self.pub_interests = rospy.Publisher('/nips2016/learning/interests', Interests, queue_size=1, latch=True)
        self.pub_focus = rospy.Publisher('/nips2016/learning/current_focus', String, queue_size=1, latch=True)
        self.pub_ready = rospy.Publisher('/nips2016/learning/ready_for_interaction', Bool, queue_size=1, latch=True)
        self.pub_iteration = rospy.Publisher('/nips2016/iteration', UInt32, queue_size=1, latch=True)

        self.dir = join(self.rospack.get_path('nips2016'), 'logs')
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)

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
        rospy.loginfo("Learning is up!")

        rate = rospy.Rate(self.params['publish_rate'])
        while not rospy.is_shutdown():
            publish = False
            with self.lock_next_iteration:
                if self.next_iteration:
                    publish = True
                    self.next_iteration = False
            if publish:
                self.publish()
            rate.sleep()

    def save(self):
        self.learning.save(self.dir, self.experiment_name)

    def publish(self):
        interests_list = self.learning.get_normalized_interests_evolution()
        interests = Interests()
        interests.names = self.learning.get_space_names()
        interests.num_iterations = UInt32(len(interests_list))
        interests.interests = [Float32(val) for sublist in interests_list for val in sublist]

        self.pub_interests.publish(interests)
        self.pub_focus.publish(String(data=self.learning.get_last_focus()))
        self.pub_ready.publish(Bool(data=self.ready_for_interaction))
        self.pub_iteration.publish(UInt32(data=self.learning.get_iterations()))


    ################################# Service callbacks
    def cb_set_iteration(self, request):
        if self.ready_for_interaction:
            self.learning.restart_from_file(self.dir, self.experiment_name, request.iteration.data)
        return SetIterationResponse()

    def cb_set_focus(self, request):
        if self.ready_for_interaction:
            self.focus = request.space.data
        return SetFocusResponse()

    def cb_perceive(self, request):
        s = self.translator.sensory_trajectory_msg_to_list(request.sensorial_demonstration)
        if len(request.torso_demonstration.points) > 0:
            torso_traj = self.translator.trajectory_msg_to_matrix(request.torso_demonstration)
            torso_traj_w = self.translator.trajectory_to_w(torso_traj)
            rospy.loginfo("Learning node is perceiving sensory + torso trajectories")
            self.learning.perceive(s, m_demo=torso_traj_w)
        else:
            rospy.loginfo("Learning node is perceiving sensory trajectory only")
            self.learning.perceive(s)

        # Regularly overwrite the results
        if self.learning.get_iterations() % self.params['save_every'] == 0:
            self.save()
        return PerceiveResponse()

    def cb_produce(self, request):
        rospy.loginfo("Learning node is requesting the current state")
        state = self.get_state(GetSensorialStateRequest()).state
        rospy.loginfo("Learning node is producing...")
        w = self.learning.produce(self.translator.get_context(state), self.focus)
        trajectory_matrix = self.translator.w_to_trajectory(w)
        trajectory_msg = self.translator.matrix_to_trajectory_msg(trajectory_matrix)
        with self.lock_next_iteration:
            self.next_iteration = True
        response = ProduceResponse(torso_trajectory=trajectory_msg)
        return response

if __name__ == '__main__':
    rospy.init_node('learning')
    LearningNode().run()

