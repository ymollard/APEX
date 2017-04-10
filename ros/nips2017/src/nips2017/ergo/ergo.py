import rospy
import rosnode
import json
from sensor_msgs.msg import Joy
from nips2017.srv import *
from nips2017.msg import *
from poppy_msgs.srv import ReachTarget, ReachTargetRequest, SetCompliant, SetCompliantRequest
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool
from rospkg import RosPack
from os.path import join
from .button import Button


class Ergo(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2017'), 'config', 'ergo.json')) as f:
            self.params = json.load(f)
        self.button = Button(self.params)
        self.rate = rospy.Rate(self.params['publish_rate'])

        # Service callers
        self.robot_reach_srv_name = '/{}/reach'.format(self.params['robot_name'])
        self.robot_compliant_srv_name = '/{}/set_compliant'.format(self.params['robot_name'])
        rospy.loginfo("Ergo node is waiting for poppy controllers...")
        rospy.wait_for_service(self.robot_reach_srv_name)
        rospy.wait_for_service(self.robot_compliant_srv_name)
        self.reach_proxy = rospy.ServiceProxy(self.robot_reach_srv_name, ReachTarget)
        self.compliant_proxy = rospy.ServiceProxy(self.robot_compliant_srv_name, SetCompliant)
        rospy.loginfo("Controllers connected!")

        self.state_pub = rospy.Publisher('/nips2017/ergo/state', CircularState, queue_size=1)
        self.button_pub = rospy.Publisher('/nips2017/ergo/button', Bool, queue_size=1)

        self.goals = []
        self.joy1_x = 0.
        self.joy1_y = 0.
        self.joy2_x = 0.
        self.joy2_y = 0.
        self.motion_started_joy = 0.
        self.js = JointState()
        rospy.Subscriber('/nips2017/sensors/joystick/1', Joy, self.cb_joy_1)
        rospy.Subscriber('/nips2017/sensors/joystick/2', Joy, self.cb_joy_2)
        rospy.Subscriber('/{}/joints'.format(self.params['robot_name']), JointState, self.cb_js)

        self.t = rospy.Time.now()
        self.srv_reset = None
        self.extended = False
        self.standby = False
        self.last_activity = rospy.Time.now()
        self.delta_t = rospy.Time.now()

    def cb_js(self, msg):
        self.js = msg

    def reach(self, target, duration):
        js = JointState()
        js.name = target.keys()
        js.position = target.values()
        self.reach_proxy(ReachTargetRequest(target=js,
                                            duration=rospy.Duration(duration)))

    def set_compliant(self, compliant):
        self.compliant_proxy(SetCompliantRequest(compliant=compliant))

    def cb_joy_1(self, msg):
        self.joy1_x = msg.axes[0]
        self.joy1_y = msg.axes[1]

    def cb_joy_2(self, msg):
        self.joy2_x = msg.axes[0]
        self.joy2_y = msg.axes[1]

    def go_to_start(self, slow=True):
        self.go_to([0.0, -15.4, 35.34, 0.0, -15.69, 71.99], 4 if slow else 1)

    def go_to_extended(self):
        extended = {'m2': 60, 'm3': -37, 'm5': -50, 'm6': 96}
        self.reach(extended, 0.5)
        self.extended = True

    def go_to_rest(self):
        rest = {'m2': -26, 'm3': 59, 'm5': -30, 'm6': 78}
        self.reach(rest, 0.5)
        self.extended = False

    def is_controller_running(self):
        return len([node for node in rosnode.get_node_names() if 'controller' in node]) > 0

    def go_or_resume_standby(self):
        recent_activity = rospy.Time.now() - self.last_activity < rospy.Duration(self.params['auto_standby_duration'])
        if recent_activity and self.standby:
            rospy.loginfo("Ergo is resuming from standby")
            self.set_compliant(False)
            self.standby = False
        elif not self.standby and not recent_activity:
            rospy.loginfo("Ergo is entering standby mode")
            self.standby = True
            self.set_compliant(True)

        if self.is_controller_running():
            self.last_activity = rospy.Time.now()

    def go_to(self, motors, duration):
        self.goals = motors
        self.reach(dict(zip(['m1', 'm2', 'm3', 'm4', 'm5', 'm6'], motors)), duration)
        rospy.sleep(duration)

    def run(self):
        self.go_to_start()
        self.last_activity = rospy.Time.now()
        self.srv_reset = rospy.Service('/nips2017/ergo/reset', Reset, self._cb_reset)
        rospy.loginfo('Ergo is ready and starts joystick servoing...')
        self.t = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.delta_t = (now - self.t).to_sec()
            self.t = now

            self.go_or_resume_standby()
            self.servo_robot(self.joy1_y, self.joy1_x)
            self.publish_state()
            self.publish_button()

            # Update the last activity
            if abs(self.joy1_x) > self.params['min_joy_activity'] or abs(self.joy1_y) > self.params['min_joy_activity']:
                self.last_activity = rospy.Time.now()

            self.rate.sleep()

    def servo_axis_rotation(self, x):
        x = x if abs(x) > self.params['sensitivity_joy'] else 0
        p = self.goals[0]
        min_x = self.params['bounds'][0][0] + self.params['bounds'][3][0]
        max_x = self.params['bounds'][0][1] + self.params['bounds'][3][1]
        new_x = min(max(min_x, p + self.params['speed']*x*self.delta_t), max_x)
        if new_x > self.params['bounds'][0][1]:
            new_x_m3 = new_x - self.params['bounds'][0][1]
        elif new_x < self.params['bounds'][0][0]:
            new_x_m3 = new_x - self.params['bounds'][0][0]
        else:
            new_x_m3 = 0
        new_x_m3 = max(min(new_x_m3, self.params['bounds'][3][1]), self.params['bounds'][3][0])
        self.reach({'m1': new_x, 'm4': new_x_m3}, 1.1/self.params['publish_rate'])

    def servo_axis_elongation(self, x):
        if x > self.params['min_joy_elongation']:
            self.go_to_extended()
        else:
            self.go_to_rest()

    def servo_robot(self, x, y):
        now = rospy.Time.now().to_sec()
        max_abs = max(abs(y), abs(x))
        if max_abs > self.params['sensitivity_joy'] and self.motion_started_joy == 0.:
            self.motion_started_joy = now

        elif max_abs < self.params['sensitivity_joy'] and self.motion_started_joy > 0.:
            self.motion_started_joy = 0.

        elif self.motion_started_joy > 0. and now - self.motion_started_joy > self.params['delay_joy']:
            if self.params['control_joystick_id'] == 2:
                self.servo_axis_rotation(-x)
                self.servo_axis_elongation(y)
            else:
                self.servo_axis_rotation(y)
                self.servo_axis_elongation(x)

    def publish_button(self):
        self.button_pub.publish(Bool(data=self.button.pressed))

    def publish_state(self):
        # TODO We might want a better state here, get the arena center, get EEF and do the maths as in environment/get_state
        if 'm1' in self.js.name and 'm4' in self.js.name:
            angle = self.js.position[0] + self.js.position[3]
            self.state_pub.publish(CircularState(angle=angle, extended=self.extended))

    def _cb_reset(self, request):
        rospy.loginfo("Resetting Ergo...")
        self.go_to_start(request.slow)
        return ResetResponse()

