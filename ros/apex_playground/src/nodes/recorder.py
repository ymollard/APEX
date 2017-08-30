#!/usr/bin/env python

from apex_playground.srv import RecordScene, RecordSceneResponse
from rospkg import RosPack
from os.path import join
from threading import RLock, Thread
import json
import cv2
import os
import rospy


class CameraRecorder(object):
    def __init__(self, parameters, duration, max_rate_hz):
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

    def read(self):
        if not self.camera or not self.camera.isOpened():
            return False, None
        success, image = self.camera.read()
        return success, image

    def record(self, experiment_name, task, condition, trial, iteration, folder="/media/usb/"):
        # Async record
        # TODO Action server
        thread = Thread(target=self._blocking_record, args=[experiment_name, task, condition, trial, iteration, folder])
        thread.daemon = True
        thread.start()

    def _blocking_record(self, experiment_name, task, condition, trial, iteration, folder):
        with self.lock:
            if self.camera is not None and self.camera.isOpened():
                self.read()  # Waste a frame
                folder_trial = os.path.join(folder, experiment_name, "task_" + str(task), "condition_" + str(condition), "trial_" + str(trial), self.name)

                if not os.path.isdir(folder_trial):
                    os.makedirs(folder_trial)

                self.writer_params['filename'] = os.path.join(folder_trial, "iteration_" + str(iteration) + self.extension)
                self.writer = cv2.VideoWriter(**self.writer_params)
                rospy.loginfo("Recording camera '{}'...".format(self.name))

            t0 = rospy.Time.now()
            try:
                while not rospy.is_shutdown():
                    now = rospy.Time.now()
                    if now < t0 + rospy.Duration(self.duration):
                        self._write()
                        self.rate.sleep()
                    else:
                        rospy.loginfo("Recorded camera '{}' in {}".format(self.name, self.writer_params['filename']))
                        break
            finally:
                if self.writer is not None:
                    self.writer.release()
                    self.writer = None

    def _write(self):
        success, image = self.read()
        if success:
            self.writer.write(image)
            return True
        return False

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

    def open(self):
        width, height = self.params['resolution']
        self.camera = cv2.VideoCapture(self.params['device'])
        if self.camera is not None and self.camera.isOpened():
            self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
            self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
            self.writer_params['frameSize'] = (int(self.camera.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)),
                                               int(self.camera.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)))

            self.read()  # First frame is longer to acquire
            # Estimate max fps
            t0 = rospy.Time.now()
            for frame in range(30):
                self.read()
            duration = rospy.Time.now() - t0
            max_fps = int(30./duration.to_sec())
            fps = min(max_fps, self.writer_params['fps'])
            # self.camera.set(cv2.cv.CV_CAP_PROP_FPS, fps)
            self.writer_params['fps'] = fps
            rospy.logwarn("Opening camera size {} {}fps".format(self.writer_params['frameSize'], fps))

    def close(self):
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

        for camera in self.cameras:
            camera.open()

    def run(self):
        rospy.Service('recorder/record', RecordScene, self.cb_record)
        rospy.spin()

    def close(self):
        for camera in self.cameras:
            camera.close()

    def cb_record(self, request):
        debug = rospy.get_param('environment/debug', False)
        experiment_name = rospy.get_param('/experiment/name')

        while not rospy.is_shutdown():
            for camera in self.cameras:
                camera.record(experiment_name, request.task, request.method, request.trial, request.iteration)
            return RecordSceneResponse(recording_duration=self.params['duration'])


        #if debug:
        #    image = Float32MultiArray()
        #    for dim in range(len(frame.shape)):
        #        image.layout.dim.append(MultiArrayDimension(size=frame.shape[dim], label=str(frame.dtype)))
        #    length = reduce(int.__mul__, frame.shape)
        #    image.data = list(frame.reshape(length))
        #    self.image_pub.publish(image)


if __name__ == '__main__':
    rospy.init_node('recorder')
    recorder = Recorder()
    try:
        recorder.run()
    finally:
        recorder.close()
