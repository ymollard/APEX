#!/usr/bin/env python
import rospy
import argparse
import json
import yaml
from threading import RLock
from os.path import join
from rospkg import RosPack
from copy import deepcopy
from apex_playground.srv import UpdateWorkStatus, GetWork, AddWork, GetWorkResponse, UpdateWorkStatusResponse, AddWorkResponse


class WorkManager(object):
    def __init__(self, outside_ros=False):
        self.rospack = RosPack()
        self.num_workers = 0
        self.outside_ros = rospy.get_param('/use_sim_time', outside_ros)  # True if work manager <-> controller comm must use ZMQ
        self.file_experiment = join(self.rospack.get_path('apex_playground'), 'config', 'experiment.json')

        with open(self.file_experiment) as f:
            experiment = self.check(json.load(f))
        rospy.set_param('/work', experiment)
        self.lock_experiment = RLock()

    @staticmethod
    def is_completed(task, trial, experiment):
        if experiment[task]['progress'][trial]['status'] == 'complete':
            return True
        return experiment[task]['progress'][trial]['iteration'] >= experiment[task]['num_iterations'] - 1

    def _cb_get_work(self, worker):
        with self.lock_experiment:
            experiment = rospy.get_param('/work')
            disabled_workers = rospy.get_param("/experiment/disabled_workers", [])
            if worker not in disabled_workers:
                for task in range(len(experiment)):
                    for trial in range(experiment[task]['num_trials']):
                        status = experiment[task]['progress'][trial]['status']
                        resuming = status =='taken' and experiment[task]['progress'][trial]['worker'] == worker
                        # If this work is open or taken and the same worker is requesting some work, distribute it!
                        if status == 'open' or resuming:
                            if self.is_completed(task, trial, experiment):
                                experiment[task]['progress'][trial]['status'] = 'complete'
                                self.num_workers -= 1
                                rospy.set_param('/work', experiment)
                            else:
                                # This task needs work, distribute it to the worker
                                experiment[task]['progress'][trial]['status'] = 'taken'
                                experiment[task]['progress'][trial]['worker'] = worker
                                rospy.set_param('/work', experiment)
                                self.num_workers += 1
                                if resuming:
                                    rospy.logwarn("Resuming worker {} {} from iteration {}/{}".format(worker, experiment[task]['method'],
                                                                                                   experiment[task]['progress'][trial]['iteration'],
                                                                                                   experiment[task]['num_iterations']))
                                else:
                                    rospy.logwarn("Distributing {} iterations {} trial {} to worker {}".format(experiment[task]['num_iterations'], experiment[task]['method'], trial, worker))
                                return dict(method=experiment[task]['method'],
                                            iteration=experiment[task]['progress'][trial]['iteration'],
                                            num_iterations=experiment[task]['num_iterations'],
                                            task=task, trial=trial, work_available=True)
            else:
                pass
                #rospy.logwarn("Worker {} requested work but it is blacklisted".format(worker))
        return dict(work_available=False)

    def _cb_update_work(self, task, trial, worker, iteration):
        inconsistent = True
        with self.lock_experiment:
            experiment = rospy.get_param('/work')
            if task > len(experiment) - 1:
                rospy.logerr("No such task: {}".format(task))
            else:
                known_status = experiment[task]['progress'][trial]['status']
                known_worker = experiment[task]['progress'][trial]['worker']
                if self.num_workers > 0:
                    if known_worker == worker or known_worker < 0:
                        if known_status == 'aborted':
                            rospy.logerr("Received aborting requested, stopping task of worker {}".format(worker))
                            return dict(abort=True)
                        elif known_status == 'taken':
                            inconsistent = False
                            experiment[task]['progress'][trial]['iteration'] = iteration
                            if self.is_completed(task, trial, experiment):
                                experiment[task]['progress'][trial]['status'] = 'complete'
                                rospy.loginfo("{} trial {} completed by worker {}".format(experiment[task]['method'], trial, worker))
                            else:
                                pass  # This is a regular update
                                rospy.loginfo("Regular update: iteration {}/{} from worker {}".format(iteration+1, experiment[task]['num_iterations'], worker))
                    if inconsistent:
                        rospy.logerr("Inconsistent data: Worker ID {} has returned task {} trial {} "
                                     "which is known {} by worker {}".format(worker, task, trial, known_status, known_worker))
                else:
                    rospy.logerr("There is no known worker, we didn't expect that work update")
        rospy.set_param('/work', experiment)
        self.save_experiment()
        return dict(abort=False)

    def _cb_add_work(self, request):
        with self.lock_experiment:
            experiment = rospy.get_param('/work')
            task = {
                    "num_iterations": request.num_iterations,
                    "num_trials": request.num_trials,
                    "method": request.method
                }
            experiment.append(task)
            rospy.set_param('/work', self.check(experiment))
            return AddWorkResponse(task=len(experiment)-1)

    def run(self):
        try:
            if self.outside_ros:
                self.run_outside_ros()
            else:
                self.run_within_ros()
        finally:
            self.save_experiment()

    def save_experiment(self):
        with self.lock_experiment:
            experiment = rospy.get_param('/work')
            with open(self.file_experiment, 'w') as f:
                json.dump(self.cleanup(deepcopy(experiment)), f, indent=4)

    def run_outside_ros(self):
        # Use ZMQ for work manager <-> controller comm
        from zmq.eventloop import zmqstream, ioloop
        from zmq import Context, REP
        rospy.logwarn('Work Manager is using ZMQ to distribute work')

        def callback_call(request):
            try:
                type = request['type']
                worker = request['worker']
                if type == 'get':
                    self.socket.send_json(self._cb_get_work(worker))
                elif type == 'update':
                    task, trial, iteration = request['task'], request['trial'], request['iteration']
                    self.socket.send_json(self._cb_update_work(task, trial, worker, iteration))
                else:
                    rospy.logerr("The Work Manager can't handle this request type: '{}'".format(type))
            except (ValueError, KeyError):
                rospy.logerr("The Work Manager can't handle this request: {}".format(request))
                self.socket.send_json({})

        self.context = Context()
        self.loop = ioloop.IOLoop.instance()
        self.socket = self.context.socket(REP)
        self.socket.bind('tcp://127.0.0.1:33589')
        #self.stream = zmqstream.ZMQStream(self.socket, self.loop)
        #self.stream.on_recv(callback_call)
        while not rospy.is_shutdown():
            callback_call(self.socket.recv_json())
        self.loop.stop()

    def run_within_ros(self):
        rospy.logwarn('Work Manager is using ROS to distribute work')

        # Use ROS for work manager <-> controller comm
        rospy.Service('work/get', GetWork, lambda req: GetWorkResponse(**self._cb_get_work(req.worker)))
        rospy.Service('work/update', UpdateWorkStatus, lambda req: UpdateWorkStatusResponse(**self._cb_update_work(req.task, req.trial, req.worker, req.iteration)))
        rospy.Service('work/add', AddWork, self._cb_add_work)

        rospy.spin()

    @staticmethod
    def check(experiment):
        # Check and prepare the experiment file
        for task in range(len(experiment)):
            if 'progress' not in experiment[task]:
                experiment[task]['progress'] = []
                for trial in range(experiment[task]['num_trials']):
                    experiment[task]['progress'].append({'iteration': 0, 'status': 'open', 'worker': -1})
        return experiment

    @staticmethod
    def cleanup(experiment):
        # Mark all taken jobs as open
        #for task in range(len(experiment)):
        #    if 'progress' in experiment[task]:
        #        for trial in range(experiment[task]['num_trials']):
        #            if experiment[task]['progress'][trial]['status'] == 'taken':
        #                experiment[task]['progress'][trial]['status'] = 'open'
        return experiment

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--comm-outside-ros",
                        help="Work distribution must occur outside ROS master on localhost:33589",
                        action="store_true")
    args, _ = parser.parse_known_args()

    rospy.init_node("work_manager")
    WorkManager(args.comm_outside_ros).run()
