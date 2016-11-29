#!/usr/bin/env python
import rospy
import pyaudio
import numpy as np
import json
from rospkg import RosPack
from std_msgs.msg import Float32
from os.path import join


class SoundController(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'environment.json')) as f:
            self.params = json.load(f)

        with open(join(self.rospack.get_path('nips2016'), 'config', 'bounds.json')) as f:
            self.bounds = json.load(f)["sensory"]["sound"][0]

        self.p = pyaudio.PyAudio()
        self.fs = 44100       # sampling rate, Hz, must be integer
        self.duration = 1./self.params['rate']

        # for paFloat32 sample values must be in range [-1.0, 1.0]
        self.stream = self.p.open(format=pyaudio.paFloat32,
                                  channels=1,
                                  rate=self.fs,
                                  output=True)

    def cb_sound(self, msg):
        value = msg.data
        if value != 0.:
            f = (value-self.bounds[0])/(self.bounds[1]-self.bounds[0]) *\
                (self.params["sound"]["freq"][1] - self.params["sound"]["freq"][0]) +\
                self.params["sound"]["freq"][0]
            self.beep(f)

    def beep(self, f):
        samples = (np.sin(2*np.pi*np.arange(self.fs*self.duration)*f/self.fs)).astype(np.float32)
        self.stream.write(samples)

    def run(self):
        rospy.Subscriber("nips2016/environment/sound", Float32, self.cb_sound)
        rospy.spin()
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

if __name__ == '__main__':
    rospy.init_node("sound_controlller")
    SoundController().run()

