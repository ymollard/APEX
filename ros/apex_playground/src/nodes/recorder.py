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


class CameraBuffer(Thread):
    def __init__(self, device, width, height, max_rate):
        super(CameraBuffer, self).__init__()
        self.setDaemon(True)
        self._rate = rospy.Rate(max_rate)
        self._error = False
        self._device = device
        self._width = width
        self._height = height
        self._image = None
        self._camera = None

    @property
    def frame(self):
        return self._image

    def _read(self, max_attempts=600):
        got_image = False
        if self._camera is not None and self._camera.isOpened():
            got_image, image = self._camera.read()

        if not got_image:
            if not self._error:
                if max_attempts > 0:
                    rospy.sleep(0.1)
                    self._open()
                    return self._read(max_attempts-1)
                rospy.logerr("Reached maximum camera reconnection attempts, abandoning!")
                self._error = True
                return False, None
            return False, None
        return True, image

    def _open(self):
        self._camera = cv2.VideoCapture(self._device)
        self._camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self._height)
        self._camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self._width)
        s, i = self._camera.read()  # First frame is longer to acquire
        self.actual_reso = (i.shape[1], i.shape[0]) if s else (0, 0)
        rospy.logwarn("Opened camera {} size {}".format(self._device, self.actual_reso))

    def _close(self):
        # cleanup the camera and close any open windows
        self._camera.release()

    def run(self):
        self._open()
        while not rospy.is_shutdown():
            success, image = self._read()
            if success:
                self._image = image
            self._rate.sleep()
        self._close()


class CameraRecorder(Thread):
    def __init__(self, parameters, duration, rate_hz):
        super(CameraRecorder, self).__init__()
        self.setDaemon(True)
        self.camera_name = parameters['name']
        self.duration = duration
        self.params = parameters
        self.rate = rospy.Rate(rate_hz)
        self.codec, self.extension = "FMP4", ".avi"
        # self.codec, self.extension = "X264", ".mp4"  # Old ffmpeg versions complain
        self.writer_params = {'filename': '', 'fourcc': cv2.cv.CV_FOURCC(*self.codec),
                              'fps': rate_hz, 'isColor': True}
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

        self.image_pub = rospy.Publisher('cameras/' + self.camera_name, Float32MultiArray, queue_size=1)

        self.camera = CameraBuffer(self.params['device'], self.params['resolution'][0], self.params['resolution'][1], rate_hz)
        self.camera.start()

    def run(self):
        while not rospy.is_shutdown():
            debug = rospy.get_param('environment/debug', False)

            now = rospy.Time.now()

            with self.stamps_lock:
                self.stamps.append(now.to_sec())

            frame = self.camera.frame

            if debug and frame is not None:
                image = Float32MultiArray()
                for dim in range(len(frame.shape)):
                    image.layout.dim.append(MultiArrayDimension(size=frame.shape[dim], label=str(frame.dtype)))
                length = reduce(int.__mul__, frame.shape)
                image.data = list(frame.reshape(length))
                self.image_pub.publish(image)

            if self.recording:
                if self.writer is None and frame is not None:
                    folder_trial = os.path.join(self.folder, self.experiment_name, "task_" + str(self.task),
                                                "condition_" + str(self.condition), "trial_" + str(self.trial), self.camera_name)

                    if not os.path.isdir(folder_trial):
                        os.makedirs(folder_trial)

                    self.writer_params['filename'] = os.path.join(folder_trial, "iteration_" + str(self.iteration) + self.extension)
                    self.writer = cv2.VideoWriter(**self.writer_params)
                    self.t0 = rospy.Time.now()
                    rospy.loginfo("Recording camera '{}' at {}fps...".format(self.camera_name, self.writer_params['fps']))
                    self.writer.write(frame)
                elif now < self.t0 + rospy.Duration(self.duration) and frame is not None:
                    self.writer.write(frame)
                elif self.writer is not None:
                    rospy.loginfo("Recorded camera '{}' in {}".format(self.camera_name, self.writer_params['filename']))
                    self.writer.release()
                    self.recording = False
                    self.writer = None
            self.rate.sleep()

    def record(self, experiment_name, task, condition, trial, iteration, folder="/media/usb/"):
        self.writer_params['frameSize'] = self.camera.actual_reso

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
        cv2.destroyAllWindows()

    def cb_record(self, request):
        experiment_name = rospy.get_param('/experiment/name', "experiment")

        for camera in self.cameras:
            camera.record(experiment_name, request.task, request.method, request.trial, request.iteration)
        return RecordSceneResponse(recording_duration=self.params['duration'])

if __name__ == '__main__':
    rospy.init_node('recorder')
    Recorder().run()
