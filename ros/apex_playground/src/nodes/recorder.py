#!/usr/bin/env python

from apex_playground.srv import RecordScene, RecordSceneResponse
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from rospkg import RosPack
from os.path import join
from threading import RLock, Thread
from collections import deque
import json
import cv2
import os
import rospy


class CameraRecorder(Thread):
    def __init__(self, parameters, duration, max_rate_hz):
        super(CameraRecorder, self).__init__()
        self.name = parameters['name']
        self.duration = duration
        self.params = parameters
        self.camera = None
        self.rate = rospy.Rate(max_rate_hz)
        self.codec, self.extension = "FMP4", ".avi"
        # self.codec, self.extension = "X264", ".mp4"  # Old ffmpeg versions complain
        self.writer_params = {'filename': '', 'fourcc': cv2.cv.CV_FOURCC(*self.codec),
                              'fps': max_rate_hz, 'isColor': True}
        self.writer = None
        self.lock = RLock()
        self.stamps_lock = RLock()
        self.stamps = deque(maxlen=30)
        self.recording = False
        self.t0 = rospy.Time(0)

        self.experiment_name = ""
        self.task = ""
        self.condition = ""
        self.trial = ""
        self.iteration = ""
        self.folder = ""

        self.image_pub = rospy.Publisher('cameras/' + self.name, Float32MultiArray, queue_size=1)

    def _read(self):
        if not self.camera or not self.camera.isOpened():
            return False, None
        success, image = self.camera.read()
        return success, image

    @property
    def avg_fps(self):
        with self.stamps_lock:
            return len(self.stamps)/(self.stamps[-1] - self.stamps[0]) if len(self.stamps) > 1 else 0

    def run(self):
        while not rospy.is_shutdown():
            debug = rospy.get_param('environment/debug', False)

            if self.camera is None or not self.camera.isOpened():
                self._open()

            success, frame = self._read()
            now = rospy.Time.now()

            if success:
                with self.stamps_lock:
                    self.stamps.append(now.to_sec())

                if debug:
                    image = Float32MultiArray()
                    for dim in range(len(frame.shape)):
                        image.layout.dim.append(MultiArrayDimension(size=frame.shape[dim], label=str(frame.dtype)))
                    length = reduce(int.__mul__, frame.shape)
                    image.data = list(frame.reshape(length))
                    self.image_pub.publish(image)

                if self.recording:
                    if self.writer is None:
                        folder_trial = os.path.join(self.folder, self.experiment_name, "task_" + str(self.task),
                                                    "condition_" + str(self.condition), "trial_" + str(self.trial), self.name)

                        if not os.path.isdir(folder_trial):
                            os.makedirs(folder_trial)

                        self.writer_params['fps'] = int(self.avg_fps)
                        self.writer_params['filename'] = os.path.join(folder_trial, "iteration_" + str(self.iteration) + self.extension)
                        self.writer = cv2.VideoWriter(**self.writer_params)
                        self.t0 = rospy.Time.now()
                        rospy.loginfo("Recording camera '{}' at {}fps...".format(self.name, self.writer_params['fps']))
                    elif now < self.t0 + rospy.Duration(self.duration):
                        self.writer.write(frame)
                    else:
                        rospy.loginfo("Recorded camera '{}' in {}".format(self.name, self.writer_params['filename']))
                        self.writer.release()
                        self.recording = False
                        self.writer = None
        self._close()

    def record(self, experiment_name, task, condition, trial, iteration, folder="/media/usb/"):
        if not self.recording:
            self.experiment_name = experiment_name
            self.task = task
            self.condition = condition
            self.trial = trial
            self.iteration = iteration
            self.folder = folder
            self.recording = True

    def draw(self, frame):
        #rgbs = cv2.split(frame)
        #hsvs = cv2.split(hsv)

        #cv2.imshow("Hue", hsvs[0])
        #cv2.imshow("Frame", frame)
        #cv2.waitKey(1)

        #cv2.imshow("Red", rgbs[0])
        #cv2.imshow("Green", rgbs[1])
        #cv2.imshow("Blue", rgbs[2])
        #cv2.imshow("Saturation", hsvs[1])
        #cv2.imshow("Value", hsvs[2])

        cv2.imshow(self.params['name'], frame)
        cv2.waitKey(1)

    def _open(self):
        width, height = self.params['resolution']
        self.camera = cv2.VideoCapture(self.params['device'])
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
        self.writer_params['frameSize'] = (int(self.camera.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)),
                                           int(self.camera.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)))

        self._read()  # First frame is longer to acquire
        rospy.logwarn("Opening camera {} size {}".format(self.name, self.writer_params['frameSize']))

    def _close(self):
        # cleanup the camera and close any open windows
        self.camera.release()
        cv2.destroyAllWindows()


class Recorder(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'recorder.json')) as f:
            self.params = json.load(f)

        self.cameras = []

        for camera in self.params['cameras']:
            self.cameras.append(CameraRecorder(camera, self.params['duration'], self.params['max_rate']))
            self.cameras[-1].start()

    def run(self):
        rospy.Service('recorder/record', RecordScene, self.cb_record)
        rospy.spin()

    def cb_record(self, request):
        experiment_name = rospy.get_param('/experiment/name')

        while not rospy.is_shutdown():
            for camera in self.cameras:
                camera.record(experiment_name, request.task, request.method, request.trial, request.iteration)
            return RecordSceneResponse(recording_duration=self.params['duration'])

if __name__ == '__main__':
    rospy.init_node('recorder')
    Recorder().run()

