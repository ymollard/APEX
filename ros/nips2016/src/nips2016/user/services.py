import rospy
from nips2016.srv import SetIteration, SetIterationRequest, SetFocus, SetFocusRequest
from nips2016.msg import Interests
from std_msgs.msg import String, Bool


class UserServices(object):
    def __init__(self):
        self.services = {'set_iteration': {'name': '/nips2016/learning/set_iteration', 'type': SetIteration},
                         'set_focus': {'name': '/nips2016/learning/set_interest', 'type': SetFocus}
                         }

        rospy.Subscriber('/nips2016/learning/interests', Interests, self._cb_interests)
        rospy.Subscriber('/nips2016/learning/current_focus', String, self._cb_focus)
        rospy.Subscriber('/nips2016/learning/ready_for_interaction', Bool, self._cb_ready)

        self.interests = {}
        self.current_focus = ""
        self.ready_for_interaction = False

        for service_name, service in self.services.items():
            rospy.loginfo("User node is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

        rospy.loginfo("User node started!")

    def _cb_interests(self, msg):
        self.interests = dict(zip(msg.names, [[msg.interests[iteration + iteration*space].data
                                               for iteration in range(msg.num_iterations.data)]
                                              for space in range(len(msg.names))]))

    def _cb_focus(self, msg):
        self.current_focus = msg.data

    def _cb_ready(self, msg):
        self.ready_for_interaction = msg.data

    def set_focus(self, space):
        call = self.services['set_focus']['call']
        return call(SetIterationRequest(space=space))

    def set_iteration(self, iteration):
        call = self.services['set_iteration']['call']
        return call(SetIterationRequest(iteration=iteration))