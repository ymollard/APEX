#!/usr/bin/env python

import rospy
import json
import numpy as np
from os.path import join, isdir
from os import system, makedirs
from rospkg.rospack import RosPack
from apex_playground.srv import *
from apex_playground.msg import SensorialState, Demonstration
from apex_playground.perception.services import PerceptionServices
from apex_playground.tools import joints, sensorial


class Perception(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'general.json')) as f:
            self.params = json.load(f)
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'torso.json')) as f:
            self.torso_params = json.load(f)
        self.rate = rospy.Rate(self.params['recording_rate'])

        # Serving these services
        self.service_name_get = "perception/get"
        self.service_name_record = "perception/record"
        # Using these services
        self.service_name_set_compliant = "{}/left_arm/set_compliant".format(self.torso_params["robot_name"])
        self.topics = PerceptionServices()  # All topics are read and stored in that object

    def run(self):
        for service in [self.service_name_set_compliant]:
            rospy.loginfo("Perception is waiting service {}".format(service))
            rospy.wait_for_service(service)
        self.set_torso_compliant_srv = rospy.ServiceProxy(self.service_name_set_compliant, SetTorsoCompliant)
        rospy.Service(self.service_name_get, GetSensorialState, self.cb_get)
        rospy.Service(self.service_name_record, Record, self.cb_record)
        rospy.loginfo("Done, perception is up!")
        rospy.spin()

    def get(self):
        state = SensorialState(ball=self.topics.ball,
                               ergo=self.topics.ergo,
                               color=self.topics.light,
                               sound=self.topics.sound,
                               joystick_1=self.topics.joy1,
                               joystick_2=self.topics.joy2,
                               hand=self.topics.torso_l_eef)
        return state     

    def wait_for_human_interaction(self, arm_threshold=1, joystick_threshold=0.15):
        rospy.loginfo("We are waiting for human interaction...")

        def detect_arm_variation():
            new_effort = np.array(self.topics.torso_l_j.effort)
            delta = np.absolute(effort - new_effort)
            return np.amax(delta) > arm_threshold

        def detect_joy_variation():
            return np.amax(np.abs(self.topics.joy1.axes)) > joystick_threshold

        effort = np.array(self.topics.torso_l_j.effort)
        rate = rospy.Rate(50)
        is_joystick_demo = None
        while not rospy.is_shutdown():
            if detect_arm_variation():
                is_joystick_demo = False
                break
            elif detect_joy_variation():
                is_joystick_demo = True
                break
            rate.sleep()
        return is_joystick_demo

    ################################# Service callbacks
    def cb_get(self, request):
        return GetSensorialStateResponse(state=self.get())

    def cb_record(self, request):
        response = RecordResponse()
        # TODO eventually keep trace of the last XX points to start recording prior to the start signal

        folder = rospy.get_param('/experiment/results_path', '/media/usb/')
        condition = rospy.get_param('experiment/current/method')
        trial = rospy.get_param('experiment/current/trial')
        task = rospy.get_param('experiment/current/task')
        iteration = rospy.get_param('experiment/current/iteration')
        experiment_name = rospy.get_param('/experiment/name')
        folder_trial = join(folder, experiment_name, "task_" + str(task), "condition_" + str(condition), "trial_" + str(trial))
        folder_traj = join(folder_trial, 'trajectories')
        
        if not isdir(folder_traj):
            makedirs(folder_traj)
        
        sensorial_points = []
        motor_points = []
        
        try:
            is_joystick_demo = False
            if request.human_demo.data:
                # Blocking... Wait for the user's grasp before recording...
                is_joystick_demo = self.wait_for_human_interaction()
                if not is_joystick_demo:
                    self.set_torso_compliant_srv(SetTorsoCompliantRequest(compliant=True))

            rospy.loginfo("Recording {}...".format("a joystick demo" if is_joystick_demo else "an arm demo"))
            for point in range(request.nb_points.data):
                if rospy.is_shutdown():
                    break
                sensorial_point = self.get()
                motor_point = self.topics.torso_l_j
 
                if point % self.params["divider_nb_points_sensory"] == 0:
                    response.demo.sensorial_demonstration.points.append(sensorial_point)
                if not is_joystick_demo:
                    response.demo.torso_demonstration.points.append(joints.state_to_jtp(motor_point))
                
                # Saving trajectories for file dumping
                sensorial_points.append(sensorial_point)
                motor_points.append(motor_point)
 
                self.rate.sleep()

            if not is_joystick_demo:
                self.set_torso_compliant_srv(SetTorsoCompliantRequest(compliant=False))

            if is_joystick_demo:
                response.demo.type_demo = Demonstration.TYPE_DEMO_JOYSTICK
                type = "demo_joy"
            elif request.human_demo.data:
                response.demo.type_demo = Demonstration.TYPE_DEMO_ARM
                type = "demo_arm"
            else:
                response.demo.type_demo = Demonstration.TYPE_DEMO_NORMAL
                type = "normal"

            trajectories = [{"motor": joints.ros_to_list(motor_points[point]),
                             "sensorial": sensorial.ros_to_dict(sensorial_points[point]),
                             "type": type} for point in range(request.nb_points.data)]

            with open(join(folder_traj, "iteration_{}.json".format(iteration)), 'w') as f:
                json.dump(trajectories, f)

        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("recording aborted!")
        except IOError:
            # TODO: total or partial iteration failure?
            raise
            rospy.logerr("I/O error when dumping trajectories")
        else:
            rospy.loginfo("Recorded!")

        return response

if __name__ == '__main__':
    rospy.init_node('perception')
    Perception().run()
