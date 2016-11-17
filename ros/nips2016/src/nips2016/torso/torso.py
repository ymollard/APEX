import rospy
import json
from nips2016.srv import *
from geometry_msgs.msg import PoseStamped
from poppy.creatures import PoppyTorso
from threading import RLock
from rospkg import RosPack
from os.path import join


class Torso(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('nips2016'), 'config', 'torso.json')) as f:
            self.params = json.load(f)

        self.publish_rate = rospy.Rate(self.params['publish_rate'])
        self.execute_rate_hz = float(self.params['trajectory_points'])/self.params['trajectory_duration']
        self.execute_rate = rospy.Rate(self.execute_rate_hz)

        self.eef_pub_l = rospy.Publisher('/nips2016/torso/left_arm/end_effector_pose', PoseStamped, queue_size=1)
        self.eef_pub_r = rospy.Publisher('/nips2016/torso/right_arm/end_effector_pose', PoseStamped, queue_size=1)

        self.srv_reset = rospy.Service('/nips2016/torso/reset', Reset, self._cb_reset)
        self.srv_execute = rospy.Service('/nips2016/torso/execute', ExecuteTorsoTrajectory, self._cb_execute)
        #self.srv_record = rospy.Service('/nips2016/torso/record', None, self._cb_record)

        # Protected resources
        self.torso = None
        self.in_rest_pose = False
        self.robot_lock = RLock()

    def go_to_rest(self, slow=False):
        with self.robot_lock:
            self.go_to([90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 4 if slow else 2)
            self.in_rest_pose = True

    def go_to(self, motors, duration):
        motors_dict = dict(zip([m.name for m in self.torso.motors], motors))
        self.torso.goto_position(motors_dict, duration)
        rospy.sleep(duration)

    def run(self, dummy=False):
        try:
            self.torso = PoppyTorso(use_http=True, simulator='poppy-simu' if dummy else None)
        except IOError as e:
            rospy.logerr("Torso failed to init: {}".format(e))
            return None

        self.torso.compliant = False
        self.go_to_rest(True)
        rospy.loginfo("Torso is ready to execute trajectories at {} Hz with joints {}".format(self.execute_rate_hz,
                                                                                              [m.name for m in self.torso.motors]))

        while not rospy.is_shutdown():
            self.publish_eef(self.torso.l_arm_chain.end_effector, self.eef_pub_l)
            self.publish_eef(self.torso.r_arm_chain.end_effector, self.eef_pub_r)
            self.publish_rate.sleep()

    def publish_eef(self, eef_pose, publisher):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = eef_pose[0]
        pose.pose.position.y = eef_pose[1]
        pose.pose.position.z = eef_pose[2]
        publisher.publish(pose)

    def _cb_execute(self, request):
        trajectory = request.torso_trajectory
        with self.robot_lock:
            if not self.in_rest_pose:
                self.go_to_rest()
            for point in trajectory.points:
                if rospy.is_shutdown():
                    break
                self.torso.goto_position(dict(zip(trajectory.joint_names, point.positions)), 1./self.execute_rate_hz)
                self.execute_rate.sleep()

    def _cb_record(self, request):
        raise NotImplementedError()

    def _cb_reset(self, request):
        with self.robot_lock:
            self.go_to_rest()
        return ResetResponse()

