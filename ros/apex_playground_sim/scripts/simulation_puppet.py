#!/usr/bin/env python

import rospy
from pypot.creatures import PoppyTorso
from apex_playground.torso.services import TorsoServices
from threading import Thread


class TorsoPuppet(object):
    """
    This node is intended to control the simulated robot by using a real robot as a joystick
    Requires Poppy Torso controllers to be connected to the simulation:
    ```
    roslaunch poppy_torso_controllers controllers.launch simulated:=true
    ```
    The real robot has a direct access via pypot and must be connected via USB to the machine running this node
    """
    def __init__(self):
        self.simulated_torso = None
        self.real_torso = None
        self.running = False

    def go_to_start(self, duration=5):
        d = {"abs_z": 0,
             "bust_y": 0,
             "bust_x": 0,
             "head_z": 0,
             "head_y": 0,
             "l_shoulder_y": 0,
             "l_shoulder_x": 0,
             "l_arm_z": 0,
             "l_elbow_y": 0,
             "r_shoulder_y": 0,
             "r_shoulder_x": 0,
             "r_arm_z": 0,
             "r_elbow_y": 0}
        self.real_torso.compliant = False
        self.simulated_torso.reach(d, duration)
        self.real_torso.goto_position(d, duration)
        rospy.sleep(duration)

    def go_to_rest(self, slow):
            duration = 2 if slow else 0.5
            self.simulated_torso.reach({'l_elbow_y': -35, 'l_shoulder_x': 30}, duration)
            self.real_torso.goto_position({'l_elbow_y': -35, 'l_shoulder_x': 30}, duration)
            rospy.sleep(duration)
            self.simulated_torso.reach({'l_shoulder_y': -25, 'l_shoulder_x': 40, 'l_arm_z': 30, 'l_elbow_y': 0}, duration)
            self.simulated_torso.goto_position({'l_shoulder_y': -25, 'l_shoulder_x': 40, 'l_arm_z': 30, 'l_elbow_y': 0}, duration)
            rospy.sleep(duration)
            rospy.sleep(0.5)

    def start(self):
        rospy.set_param('demo_mode', False)
        self.simulated_torso = TorsoServices("poppy_torso")
        self.real_torso = PoppyTorso()
        try:
            Thread(target=self.puppet).start()
            for m in self.real_torso.motors:
                m.torque_limit = 50

            self.go_to_start()
            self.run()
        finally:
            if self.real_torso is not None:
                self.real_torso.close()

    def puppet(self):
        while not rospy.is_shutdown() and self.running:
            target = {m.name : m.present_position for m in self.real_torso.motors}
            self.simulated_torso.reach(target, 0.05)
            rospy.sleep(0.05)

    def run(self):
        self.running = True
        while not rospy.is_shutdown() and self.running:
            text = raw_input("Do you want to set the real robot compliant? "
                             "(left arm, right arm, both arms, not compliant, go to start pose, quit) [L/R/B/N/S/Q] ").lower()
            if text not in ['l', 'r', 'b', 'n', 's', 'q']:
                rospy.logerr("Invalid option: {}".format(text))
            else:
                if text == 'l':
                    for m in self.real_torso.l_arm:
                        m.compliant = True
                elif text == 'r':
                    for m in self.real_torso.r_arm:
                        m.compliant = True
                elif text == 'b':
                    for m in self.real_torso.l_arm:
                        m.compliant = True
                    for m in self.real_torso.r_arm:
                        m.compliant = True
                elif text == 'n':
                    self.real_torso.compliant = False
                elif text == 's':
                    self.real_torso.compliant = False
                    self.go_to_start()
                elif text == 'q':
                    rospy.loginfo("Quitting...")
                    self.running = False

if __name__ == '__main__':
    rospy.init_node('torso_puppet')
    TorsoPuppet().start()
