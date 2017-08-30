from apex_playground.srv import RecordSceneRequest, RecordScene
import rospy


class Recorder(object):
    def __init__(self):
        self.services = {'record': {'name': 'recorder/record', 'type': RecordScene}}

        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def record(self, task, method, trial, iteration):
        call = self.services['record']['call']
        return call(RecordSceneRequest(task=task, method=method, trial=trial, iteration=iteration))