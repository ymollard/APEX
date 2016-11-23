#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import random
import rospy
from os.path import join
from flask import Flask
from flask import request
from flask_cors import CORS
from rospkg import RosPack
from nips2016.user import UserServices


class UserNode(object):
    def __init__(self):
        self.rospack = RosPack()
        self.web_app_root = join(self.rospack.get_path('nips2016'), 'webapp')
        self.app = Flask(__name__, static_url_path=self.web_app_root)
        self.cors = CORS(self.app, resources={r'/api/*': {'origins': '*'}})
        #self.services = UserServices()

    def route(self):
        self.app.route('/', callback=self.root)
        self.app.route('/api/interests', methods=['GET'], callback=self.experiment_status)
        self.app.route('/api/focus', methods=['POST'], callback=self.update_focus)
        self.app.route('/api/time-travel', methods=['POST'], callback=self.time_travel)
        self.app.route('/api/reset', methods=['POST'], callback=self.reset)

    def generate_dummy_scores(self, num_interests=7):
        max_score = 1
        scores = []
        for i in range(0, num_interests):
            rdm_score = round(max_score * random.random(), 2)
            scores.append(rdm_score)
            max_score = max_score - rdm_score
        return scores

    def root(self):
        return self.app.send_static_file('index.html')

    def experiment_status(self):
        """ Returns default experiment status. """
        dummy_scores = self.generate_dummy_scores()

        return json.dumps({
            'isBusy': False,
            'interests': [
                {'interestId': 's_hand', 'value': dummy_scores[0], 'title': 'Hand'},
                {'interestId': 's_joystick_1', 'value': dummy_scores[1], 'title': 'Joystick Left'},
                {'interestId': 's_joystick_2', 'value': dummy_scores[2], 'title': 'Joystick Right'},
                {'interestId': 's_ergo', 'value': dummy_scores[3], 'title': 'Ergo'},
                {'interestId': 's_ball', 'value': dummy_scores[4], 'title': 'Ball'},
                {'interestId': 's_light', 'value': dummy_scores[5], 'title': 'Light'},
                {'interestId': 's_sound', 'value': dummy_scores[6], 'title': 'Sound'}
            ],
            'dataset': [
                {
                    'interestId': 's_hand',
                    'data': [2,6.1,9.7,8.6,6.6,8.7,8.1,9.3,1.6,7.2,4.2,7.4,1.9,1.3,9.9,6.4,4.6,1.7,9.3,6.9,0.2,5.9,6.8,1.1,0.4,5.5,8.2,0.3,8,0.2,3.2,8,0.2,3.7,0.7,5.7,0.9,9.4,10,10,5.7,5.8,6.6,7.6,7.3,0.4,0.1,7.7,0.6,7.1,1.2,9.8,8.2,0.5,2,7.6,7.9,8.4,5.1,5.8,4.9,9.6,1.9,8.9,7.6,9.8,0.6,7.8,1.7,9.7,5.9,2.2,7.2,1.8,6.7,5.9,0.1,4.1,1,0.6,1.8,0.4,0.8,2.5,0.5,1,2.5,5.2,0.8,0.2,2.1,0.6,4.7,0.9,2.2,8.2,9.2,5.7,3.7,7.1]
                },
                {
                    'interestId': 's_ball',
                    'data': [2.7,6,5.7,0.2,3.4,8.3,4.3,4.2,7.7,3.5,1.3,1.8,4.4,8.8,8.1,6.2,6.6,9,6.4,8.5,2.8,0.6,2.1,2.4,6.3,8.3,8.4,9.2,4.3,7.7,7.4,3.2,3,0.8,1.3,5,6.5,0,6.8,1.8,1.4,0.9,4.9,0.8,9.4,6.6,6.8,7.2,2.5,3.3,8.6,0.3,2.3,2.1,1.8,8.7,2.5,0.6,0.4,7.6,1,3.4,2.6,4.7,4.9,9.8,10,6.8,4.7,8.4,2.3,7.1,7.6,7.9,3.4,1.4,6.8,4.6,3.9,3.7,1.3,7.9,2.3,4.6,2,3.9,2.3,1,7.5,2,8.3,7.5,4.1,9.2,0.8,9.9,8.6,1.5,9.3,4.4]
                },
                {
                    'interestId': 's_joystick_1',
                    'data': [2.8,5.6,9.6,6.2,2.9,7.5,3.8,3.9,6.8,4.2,9.5,4.6,2.1,6.3,3.2,0.6,1.6,8.8,9.2,3.7,1.7,5.2,6.2,1,9.7,3.7,0.4,8.6,8.7,7.5,4.6,7.6,4.2,1.2,5.7,5.2,4.1,5,1.7,8.1,1.6,2.1,1.7,0.1,1.4,0.9,8.3,3,7.9,7,7.1,3.1,6.4,4.7,1.3,4.8,8.9,9.1,2.1,5.5,5.2,4.8,4,1.1,4.4,3.5,6.2,5.4,5.8,1.8,1.4,8.3,9.8,7,2.6,1.1,8,9.7,1.4,3.3,9.3,0.5,6.1,4.6,4.5,3,9.6,3.8,0.1,6.7,7.6,2.2,2.8,9.2,5.9,1.9,6,3.2,0.3,6.7]
                },
                {
                    'interestId': 's_joystick_2',
                    'data': [0.9,8.9,3.5,9.5,8,8.6,3.6,1.4,4,2.1,3,2.1,0.1,9.6,9.8,4.5,3.4,0.1,1.1,8.2,8,5.4,8.3,1.1,7.1,0.2,4.9,1.3,2.7,0.2,9.6,6.1,1.5,0.9,3.8,8.5,1.5,1.9,6,3.2,4.3,2.2,5.6,0.8,8.8,8.9,2.6,6.2,0.4,2.2,6.3,0.8,3,7.5,7,5.6,4.7,6.1,9.5,2.2,9.9,1.3,1.4,0.7,3.6,5.6,2.1,5,0.6,3.9,0.3,0.2,3.4,5.3,4.1,0.3,5.2,1.2,0.9,2.5,9.7,8.1,2.5,5.4,8.2,1.8,3.9,1.5,3.4,4.5,0.6,3.6,4.4,5.8,5.4,1,2,1.4,7.9,0.2]
                }
            ]
        })

    def update_focus(self):
        """ Updates focused interest."""
        try:
            interest_id = request.form['interestId']
            print interest_id
            return ('', 204)
        except Exception as e:
            raise

    def time_travel(self):
        """ Revert experiment state to a given point."""
        try:
            point = request.form['point']
            print point
            return ('', 204)
        except Exception as e:
            raise

    def reset(self, arg):
        pass

    def run(self):
        self.route()
        self.app.run(port=9856)
        # rospy.spin()

if __name__ == '__main__':
    rospy.init_node('user')
    UserNode().run()
