import os
import sys
import cPickle
import numpy as np



# PARAMS
filename = "/home/sforesti/ros/home/ros/Repos/NIPS2016/ros/nips2016/logs/experiment.pickle"
filename = "/home/sforesti/ros/home/ros/Repos/NIPS2016/ros/nips2016/logs/experiment_AMB_0.pickle"

with open(filename, 'r') as f:
    log = cPickle.load(f)
f.close()

n = len(log["sm_data"]["mod2"][1])

#print log["sm_data"]["mod2"][1]

j_touch = 0
b_touch = 0

for i in range(n):
    if np.linalg.norm(log["sm_data"]["mod2"][1][i] - np.array([-1.,  0., -1.,  0., -1.,  0., -1.,  0., -1.,  0., -1.,  0., -1., 0., -1.,  0., -1.,  0., -1.,  0.])) > 0.01:
        j_touch += 1
        print
        print "joystick", list(log["sm_data"]["mod2"][1][i])
        if abs(list(log["sm_data"]["mod5"][1][i])[2:][0] -  list(log["sm_data"]["mod5"][1][i])[2:][-2]) > 0.05:
            print "ball angle", list(log["sm_data"]["mod5"][1][i])[2:]
            b_touch += 1

print "Motor Babbling Iterations:", n
print "Joystick touched:", j_touch, "percentage:", int(100. * j_touch / n), "%"
print "Ball touched:", b_touch, "percentage:", int(100. * b_touch / n), "%"