#!/usr/bin/env python

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from apex_playground.srv import RecordScene, RecordSceneResponse
from rospkg import RosPack
from os.path import join
import json
import cv2
import os
import rospy


class CameraRecorder(object):
    def __init__(self, parameters, rate):
        self.name = parameters['name']
        self.params = parameters
        self.camera = None
        self.rate = rate
        self.writer_params = {'filename': '', 'fourcc': cv2.cv.CV_FOURCC(*'H264'),
                              'fps': self.rate, 'isColor': True}
        self.writer = None

    def read(self):
        if not self.camera:
            return False, None
        success, image = self.camera.read()
        return success, image

    def record(self, experiment_name, task, condition, trial, iteration, folder="/media/usb/"):
        folder_trial = os.path.join(folder, experiment_name, "task_" + task, "condition_" + condition, "trial_" + trial, self.name)

        if not os.path.isdir(folder_trial):
            os.makedirs(folder_trial)

        self.writer_params['filename'] = "iteration_" + str(iteration) + ".mp4"
        self.writer = cv2.VideoWriter(**self.writer_params)

    def write(self):
        success, image = self.read()
        if success:
            self.writer.write(image)

    def save(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None

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
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)

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
            self.cameras.append(CameraRecorder(camera, self.params['rate']))

    def run(self):

        for camera in self.cameras:
            camera.open()

        rospy.Service('recorder/record', RecordScene, self.cb_record)
        rospy.spin()

    def cb_record(self, request):
        debug = rospy.get_param('environment/debug', False)
        experiment_name = rospy.get_param('experiment/name')
        t0 = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            for camera in self.cameras:
                camera.record(experiment_name, request.task, request.condition, request.trial, request.iteration)
                try:
                    for camera in self.cameras:
                        if now < t0 + self.params['duration']:
                            camera.write()
                        else:
                            break
                finally:
                    camera.save()
                rospy.loginfo("Recorder successfully saved camera {}".format(camera.name))
            return RecordSceneResponse()


        #if debug:
        #    image = Float32MultiArray()
        #    for dim in range(len(frame.shape)):
        #        image.layout.dim.append(MultiArrayDimension(size=frame.shape[dim], label=str(frame.dtype)))
        #    length = reduce(int.__mul__, frame.shape)
        #    image.data = list(frame.reshape(length))
        #    self.image_pub.publish(image)


if __name__ == '__main__':
    rospy.init_node('recorder')
    Recorder().run()
