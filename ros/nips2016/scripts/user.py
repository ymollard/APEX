#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import rospy
from os.path import join
from flask import Flask
from flask import request
from flask_cors import CORS
from rospkg import RosPack
from rospy import ServiceException
from nips2016.user import UserServices


class UserNode(object):
    def __init__(self):
        self.rospack = RosPack()
        self.web_app_root = join(self.rospack.get_path('nips2016'), 'webapp', 'static')
        self.app = Flask(__name__, static_url_path='', static_folder=self.web_app_root)
        self.cors = CORS(self.app, resources={r'/api/*': {'origins': '*'}})
        self.services = UserServices()

        self.app.route('/')(self.root)
        self.app.route('/api/interests', methods=['GET'])(self.experiment_status)
        self.app.route('/api/focus', methods=['POST'])(self.update_focus)
        self.app.route('/api/time-travel', methods=['POST'])(self.time_travel)
        self.app.route('/api/reset', methods=['POST'])(self.reset)

    # def generate_dummy_scores(self, num_interests=7):
    #     max_score = 1
    #     scores = []
    #     for i in range(0, num_interests):
    #         rdm_score = round(max_score * random.random(), 2)
    #         scores.append(rdm_score)
    #         max_score = max_score - rdm_score
    #     return scores

    def root(self):
        return self.app.send_static_file('index.html')

    def experiment_status(self):
        """ Returns default experiment status. """
        scores = self.services.interests

        return json.dumps({
            'isBusy': False,
            'interests': [
                {'interestId': 's_hand', 'value': scores['s_hand'][-1], 'title': 'Hand'},
                {'interestId': 's_joystick_1', 'value': scores['s_joystick_1'][-1], 'title': 'Joystick Left'},
                {'interestId': 's_joystick_2', 'value': scores['s_joystick_2'][-1], 'title': 'Joystick Right'},
                {'interestId': 's_ergo', 'value': scores['s_ergo'][-1], 'title': 'Ergo'},
                {'interestId': 's_ball', 'value': scores['s_ball'][-1], 'title': 'Ball'},
                {'interestId': 's_light', 'value': scores['s_light'][-1], 'title': 'Light'},
                {'interestId': 's_sound', 'value': scores['s_sound'][-1], 'title': 'Sound'}
            ],
            'dataset': [
                {
                    'interestId': 's_hand',
                    'data': scores['s_hand']
                },
                {
                    'interestId': 's_ball',
                    'data': scores['s_ball']
                },
                {
                    'interestId': 's_joystick_1',
                    'data': scores['s_joystick_1']
                },
                {
                    'interestId': 's_joystick_2',
                    'data': scores['s_joystick_2']
                }
            ]
        })

    def update_focus(self):
        """ Updates focused interest."""
        try:
            interest_id = request.form['interestId']
            self.services.set_focus(interest_id)
            return '', 204
        except ServiceException as e:
            rospy.logerr("Cannot set focus. " + repr(e))

    def time_travel(self):
        """ Revert experiment state to a given point."""
        try:
            point = request.form['point']
            self.services.set_iteration(point)
            return '', 204
        except ServiceException as e:
            rospy.logerr("Cannot set iteration. " + repr(e))

    def reset(self, arg):
        pass

    def run(self):
        rospy.loginfo("User node is serving the Web app")
        self.app.run()
        # rospy.spin()

if __name__ == '__main__':
    rospy.init_node('user')
    UserNode().run()
