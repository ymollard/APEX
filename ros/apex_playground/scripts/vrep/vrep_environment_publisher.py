#!/usr/bin/env python

from sensor_msgs.msg import Joy
from numpy import sin, array
from numpy.linalg import norm
from rospkg import RosPack
from os.path import join
from apex_playground.msg import CircularState
from apex_playground.environment.conversions import EnvironmentConversions
from std_msgs.msg import UInt8, Float32
from std_srvs.srv import Empty, EmptyResponse
from time import sleep
import pypot.vrep.remoteApiBindings.vrep as vrep # Pypot has the path to V-Rep bindings, but that could also be a direct import
import rospy
import json
import ctypes


class JointTracker(object):
    def __init__(self, joints, simulation_id=0):
        self._joints = {}
        self.simulation_id = simulation_id
        for joint in joints:
            self._joints[joint] = {'initialized': False}
            error, handle = vrep.simxGetObjectHandle(self.simulation_id, joint, vrep.simx_opmode_blocking)
            if error == 0:
                self._joints[joint]["handle"] = handle
        self.update()

    def update(self):
        for joint in self._joints:
            if 'handle' in self._joints[joint]:
                error, pos = vrep.simxGetJointPosition(self.simulation_id, self._joints[joint]['handle'],
                                                       vrep.simx_opmode_buffer if self._joints[joint]['initialized'] else vrep.simx_opmode_streaming)

                if error == 0:
                    self._joints[joint]["position"] = pos

    def get(self):
        return self._joints


class ObjectTracker(object):
    def __init__(self, objects, simulation_id=0):
        self._objects = {}
        self.simulation_id = simulation_id
        for obj in objects:
            self._objects[obj] = {'initialized': False}
            error, handle = vrep.simxGetObjectHandle(self.simulation_id, obj, vrep.simx_opmode_blocking)
            if error == 0:
                self._objects[obj]["handle"] = handle
        self.base = -1  # Will return poses in world frame
        self.update()

    def update(self):
        for obj in self._objects:
            if 'handle' in self._objects[obj]:
                error, pos = vrep.simxGetObjectPosition(self.simulation_id, self._objects[obj]['handle'], self.base,
                                                        vrep.simx_opmode_buffer if self._objects[obj]['initialized'] else vrep.simx_opmode_streaming)
                if error == 0:
                    self._objects[obj]["position"] = pos
                    self._objects[obj]['initialized'] = True

    def get(self):
        return self._objects


class VRepEnvironmentPublisher(object):
    def __init__(self):
        self.joy_pub = rospy.Publisher('sensors/joystick/1', Joy, queue_size=1)
        self.joy_pub2 = rospy.Publisher('sensors/joystick/2', Joy, queue_size=1)
        self.ball_pub = rospy.Publisher('environment/ball', CircularState, queue_size=1)
        self.light_pub = rospy.Publisher('environment/light', UInt8, queue_size=1)
        self.sound_pub = rospy.Publisher('environment/sound', Float32, queue_size=1)

        self.rospack = RosPack()
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'ergo.json')) as f:
            self.ergo_params = json.load(f)
        with open(join(self.rospack.get_path('apex_playground'), 'config', 'environment.json')) as f:
            self.env_params = json.load(f)

        self.rate = rospy.Rate(self.ergo_params['publish_rate'])
        vrep_port = rospy.get_param('vrep/environment_port', 29997)
        self.simulation_id = vrep.simxStart('127.0.0.1', vrep_port, True, False, 5000, 5)

        # Object names in V-Rep
        self.joystick_left_joints = ['Joystick_2_Axis_2', 'Joystick_2_Axis_1']
        self.joystick_right_joints = ['Joystick_1_Axis_2', 'Joystick_1_Axis_1']
        self.ball_name = 'TennisBall'
        self.arena_name = 'Arena'

        if self.ergo_params["control_joystick_id"] != 2:
            useless_joy = self.joystick_left_joints
            self.joystick_left_joints = self.joystick_right_joints
            self.joystick_right_joints = useless_joy

        self.joints = JointTracker(self.joystick_left_joints + self.joystick_right_joints, self.simulation_id)
        self.objects = ObjectTracker([self.ball_name, self.arena_name], self.simulation_id)
        self.conversions = EnvironmentConversions()

    def cb_reset_ball(self, req):
        rospy.logwarn("Resetting dynamic object: ball")
        self.reset_ball()
        return EmptyResponse()

    def _inject_lua_code(self, lua_code):
        # This works with the according LUA script running on the client side
        msg = (ctypes.c_ubyte * len(lua_code)).from_buffer_copy(lua_code.encode())
        vrep.simxWriteStringStream(self.simulation_id, 'my_lua_code', msg, vrep.simx_opmode_oneshot)

    def start_simulation(self):
        vrep.simxStartSimulation(self.simulation_id, vrep.simx_opmode_oneshot)

    def stop_simulation(self):
        vrep.simxStopSimulation(self.simulation_id, vrep.simx_opmode_oneshot)

    def reset_ball(self):
        ball = self.objects.get()[self.ball_name]
        if 'handle' in ball:
            position = "{ 0.1759, -0.068, 0.7697 }"
            self._inject_lua_code("simSetObjectPosition({}, -1 , {})".format(ball['handle'], position))
            self._inject_lua_code("simResetDynamicObject({})".format(ball['handle']))

    def publish_joy(self, x, y, publisher):
        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.axes.append(x)
        joy.axes.append(y)
        publisher.publish(joy)

    def run(self):
        self.start_simulation()
        rospy.Service('vrep/reset_ball', Empty, self.cb_reset_ball)

        try:
            self.reset_ball()
            while not rospy.is_shutdown():
                self.joints.update()
                self.objects.update()

                joints = self.joints.get()

                def map_joy(x):
                    h_joy = 2.
                    return min(max(h_joy*sin(x), -1), 1)

                x = map_joy(joints[self.joystick_left_joints[0]]['position'])
                y = map_joy(joints[self.joystick_left_joints[1]]['position'])

                # Publishers
                self.publish_joy(x, y, self.joy_pub)
                x = map_joy(joints[self.joystick_right_joints[0]]['position'])
                y = map_joy(joints[self.joystick_right_joints[1]]['position'])
                self.publish_joy(x, y, self.joy_pub2)

                objects = self.objects.get()
                if 'position' in objects[self.ball_name] and 'position' in objects[self.arena_name]:
                    ring_radius = self.env_params['tracking']['arena']['radius'] / self.env_params['tracking']['ring_divider']     # There is no visual tracking so the true arena diameter is assumed
                    ball_state = self.conversions.get_state(objects[self.ball_name]['position'],  # Although returned by V-Rep, dimension "z" is ignored
                                                            objects[self.arena_name]['position'],
                                                            ring_radius)
                    self.ball_pub.publish(ball_state)

                    color = self.conversions.ball_to_color(ball_state)
                    self.light_pub.publish(UInt8(color))

                    sound = self.conversions.ball_to_sound(ball_state)
                    self.sound_pub.publish(Float32(sound))

                    distance = norm(array(objects[self.ball_name]['position']) - array(objects[self.arena_name]['position']))
                    if distance > 0.25:
                        self.reset_ball()

                self.rate.sleep()
        finally:
            self.stop_simulation()
            sleep(0.5)
            vrep.simxFinish(self.simulation_id)

if __name__ == '__main__':
    rospy.init_node('vrep_environment_publisher')
    VRepEnvironmentPublisher().run()
