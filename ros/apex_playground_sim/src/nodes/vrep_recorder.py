#!/usr/bin/env python

from apex_playground.srv import RecordScene, RecordSceneResponse
from rospkg import RosPack
from os.path import join

import json
import rospy


class Recorder(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'recorder.json')) as f:
            self.params = json.load(f)

    def run(self):
        rospy.Service('recorder/record', RecordScene, self.cb_record)
        rospy.spin()

    def cb_record(self, request):
        rospy.logerr('VRep camera recorder is fake, thus corresponding service is void')
        return RecordSceneResponse(recording_duration=self.params['duration'])

if __name__ == '__main__':
    rospy.init_node('recorder')
    Recorder().run()
