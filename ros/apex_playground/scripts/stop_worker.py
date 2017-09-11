#!/usr/bin/env python

import rospy
import argparse
import sys
from apex_playground.srv import AddWork, AddWorkRequest

rospy.init_node('stop_worker')

parser = argparse.ArgumentParser(description="Abort the current task being run by the worker")
parser.add_argument("--blacklist", action="store_true", help="Abort but also blacklist the worker")
parser.add_argument("--redistribute", action="store_true", help="Create a new task to redistribute the aborted trial on a different worker, if any")
parser.add_argument("--worker", required=True, type=int, help="ID of the worker to be stopped")
args = parser.parse_args()

worker_known = rospy.get_param('/apex_{}'.format(args.worker), None) is not None

if not worker_known:
    rospy.logerr("Worker {} is not currently active".format(args.worker))

if args.blacklist:
    disabled_workers = rospy.get_param("/experiment/disabled_workers", [])
    
    if args.worker not in disabled_workers:
        disabled_workers.append(args.worker)
        rospy.set_param("/experiment/disabled_workers", disabled_workers)
        rospy.logwarn("Worker {} successfully blacklisted, please unplug its power supply".format(args.worker))
    else:
        rospy.loginfo("Worker {} already blacklisted".format(args.worker))


experiment = rospy.get_param("/work")
for task in range(len(experiment)):
    for trial in range(experiment[task]['num_trials']):
        status = experiment[task]['progress'][trial]['status']
        if experiment[task]['progress'][trial]['worker'] == args.worker:
            if status == 'taken':
                experiment[task]['progress'][trial]['status'] = 'aborted'
                rospy.set_param("/work", experiment)
                rospy.loginfo("Sucessfully requested abortion of the current task of worker {}".format(args.worker))

                if args.redistribute:
                    num_iterations = experiment[task]['num_iterations']
                    method = experiment[task]['method']
                    
                    rospy.wait_for_service('/work/add')
                    add_work = rospy.ServiceProxy('/work/add', AddWork)
                    result = add_work(AddWorkRequest(method=method, num_iterations=num_iterations, num_trials=1))
                    if result.task > 0:
                        rospy.loginfo("Sucessfully redistributed 1 trial of {} {} iterations as task {}".format(method, num_iterations, result.task))
