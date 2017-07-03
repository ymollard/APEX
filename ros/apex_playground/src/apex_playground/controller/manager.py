import rospy
from zmq import Context, REQ
from apex_playground.srv import UpdateWorkStatus, GetWork, GetWorkRequest, UpdateWorkStatusRequest, GetWorkResponse, UpdateWorkStatusResponse


class WorkManager(object):
    def __init__(self, worker_id, outside_ros=False):
        self.worker_id = worker_id
        self.outside_ros = outside_ros
        if self.outside_ros:
            rospy.logwarn('Controller is using ZMQ to get work')
            self.context = Context()
            self.socket = self.context.socket(REQ)
            self.socket.connect('tcp://127.0.0.1:33589')
        else:
            rospy.logwarn('Controller is using ROS to get work')

            self.services = {'get': {'name': '/work/get', 'type': GetWork},
                             'update': {'name': '/work/update', 'type': UpdateWorkStatus}}
            for service_name, service in self.services.items():
                rospy.loginfo("Controller is waiting service {}...".format(service['name']))
                rospy.wait_for_service(service['name'])
                service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def get(self):
        if self.outside_ros:
            self.socket.send_json({"type": "get", "worker": self.worker_id})
            return GetWorkResponse(**self.socket.recv_json())
        else:
            call = self.services['get']['call']
            return call(GetWorkRequest(worker=self.worker_id))

    def update(self, task, trial, iteration):
        if self.outside_ros:
            self.socket.send_json({"type": "update", "task": task, "trial": trial, "iteration": iteration, "worker": self.worker_id})
            return UpdateWorkStatusResponse(**self.socket.recv_json())
        else:
            call = self.services['update']['call']
            return call(UpdateWorkStatusRequest(task=task, trial=trial, iteration=iteration,
                                                worker=self.worker_id))

