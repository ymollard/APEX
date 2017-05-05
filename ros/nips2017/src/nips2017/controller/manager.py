import rospy
from nips2017.srv import UpdateWorkStatus, GetWork, GetWorkRequest, UpdateWorkStatusRequest


class WorkManager(object):
    def __init__(self, worker_id):
        self.worker_id = worker_id
        self.services = {'get': {'name': 'work/get', 'type': GetWork},
                         'update': {'name': 'work/update', 'type': UpdateWorkStatus}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def get(self):
        call = self.services['get']['call']
        return call(GetWorkRequest(worker=self.worker_id))

    def update(self, task, trial, iteration, is_dying=False):
        call = self.services['update']['call']
        return call(UpdateWorkStatusRequest(task=task, trial=trial, iteration=iteration,
                                            isDying=is_dying, worker=self.worker_id))

