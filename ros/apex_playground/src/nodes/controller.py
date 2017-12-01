#!/usr/bin/env python
import rospy
import json
from os.path import join
from rospkg import RosPack
from apex_playground.controller import Perception, Learning, Torso, Ergo, WorkManager, Recorder
from apex_playground.srv import Assess, AssessResponse
from trajectory_msgs.msg import JointTrajectory
from re import search


class Controller(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'general.json')) as f:
            self.params = json.load(f)

        with open(join(self.rospack.get_path('apex_playground'), 'config', 'torso.json')) as f:
            self.torso_params = json.load(f)

        self.outside_ros = rospy.get_param('/use_sim_time', False)  # True if work manager <-> controller comm must use ZMQ
        id = search(r"(\d+)", rospy.get_namespace())
        self.worker_id = 0 if id is None else int(id.groups()[0])  # TODO string worker ID
        self.work = WorkManager(self.worker_id, self.outside_ros)
        self.torso = Torso()
        self.ergo = Ergo()
        self.learning = Learning()
        self.perception = Perception()
        self.recorder = Recorder()
        self.demonstrate = ""  # Skill (Target space for Produce) or empty string if not running assessment

        # Served services
        self.service_name_demonstrate = "controller/assess"
        rospy.Service(self.service_name_demonstrate, Assess, self.cb_assess)
        
        rospy.loginfo('Controller fully started!')

    def run(self):
        work = self.work.get()
        if not work.work_available:
            return

        rospy.set_param('experiment/current/task', work.task)
        rospy.set_param('experiment/current/trial', work.trial)
        rospy.set_param('experiment/current/method', work.method)

        iteration = work.iteration
        success = False
        while iteration < work.num_iterations and not rospy.is_shutdown():
            if iteration % self.params["ergo_reset"] == 0:
                self.ergo.reset()
            try:
                rospy.set_param('experiment/current/iteration', iteration)
                if self.perception.has_been_pressed('buttons/pause') or self.demonstrate != "":
                    # If assessment requested but not already in pause, entering pause mode anyway
                    while not rospy.is_shutdown() and not self.perception.has_been_pressed('buttons/pause'):
                        rospy.set_param('experiment/current/iteration', iteration)  # Update iteration even in pause/assess mode
                        if self.demonstrate != "":
                            # Assessment has been asked by user, space is self.demonstrate
                            success = self.execute_iteration(work.task, work.method, iteration, work.trial, work.num_iterations)
                            if success:
                                iteration += 1
                            self.demonstrate = ""
                        self.perception.switch_led('button_leds/pause')
                        rospy.logwarn("Controller is paused, pressed pause button to resume...")
                        rospy.sleep(0.5)
                else:
                    success = self.execute_iteration(work.task, work.method, iteration, work.trial, work.num_iterations)
                    if success:
                        iteration += 1
            #except IndexError, IOError:
                #pass
                # TODO: Rewind 1 iteration
            finally:
                self.perception.switch_led('button_leds/pause', False)
                update = self.work.update(work.task, work.trial, iteration, success)
                if update.abort:
                    rospy.logwarn("Work manager requested abortion, closing...")
                    return
        rospy.loginfo("Work successfully terminated, closing...")

    def execute_iteration(self, task, method, iteration, trial, num_iterations):
        rospy.logwarn("Controller starts iteration {} {}/{} trial {}".format(method, iteration, num_iterations, trial))
        rospy.wait_for_service('ergo/reset')  # Ensures Ergo keeps working or wait till it reboots
       
        # After resuming, we keep the same iteration
        if self.perception.has_been_pressed('buttons/help'):
            rospy.sleep(1.5)  # Wait for the robot to fully stop
            self.recorder.record(task, method, trial, iteration)
            self.perception.switch_led('button_leds/pause', True)
            recording = self.perception.record(human_demo=True, nb_points=self.params['nb_points'])
            self.torso.set_torque_max(self.torso_params['torques']['reset'])
            self.torso.reset(slow=True)
            return True
        else:
            trajectory = self.learning.produce(skill_to_demonstrate=self.demonstrate).torso_trajectory
            self.torso.set_torque_max(self.torso_params['torques']['motion'])
            self.recorder.record(task, method, trial, iteration) 
            self.perception.switch_led('button_leds/pause', True)
            self.torso.execute_trajectory(trajectory)  # TODO: blocking, non-blocking, action server?
            recording = self.perception.record(human_demo=False, nb_points=self.params['nb_points'])
            self.perception.switch_led('button_leds/pause', False)
            recording.demo.torso_demonstration = JointTrajectory()
            self.torso.set_torque_max(80)
            self.torso.reset(slow=False)
            return self.learning.perceive(recording.demo)

    def cb_assess(self, request):
        self.demonstrate = request.goal
        return AssessResponse()


if __name__ == '__main__':
    rospy.init_node("controller")
    Controller().run()
