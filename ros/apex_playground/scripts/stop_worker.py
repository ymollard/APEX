#!/usr/bin/env python

import rospy
import argparse
import sys

rospy.init_node('stop_worker')

parser = argparse.ArgumentParser(description="Abort the current task being run by the worker")
parser.add_argument("--blacklist", action="store_true", help="Abort but also blacklist the worker")
parser.add_argument("--worker", type=int, help="ID of the worker to be stopped, among [1, 6]")
args = parser.parse_args()

if args.worker not in range(1, 7):
    raise ValueError("Please provide a worker ID among [1, 6]")

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
