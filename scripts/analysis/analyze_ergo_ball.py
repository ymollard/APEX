import os
import sys
import cPickle
import numpy as np
import matplotlib.pyplot as plt


# PARAMS
filename = "/home/sforesti/ros/home/ros/Repos/NIPS2016/ros/nips2016/logs/experiment.pickle"
filename = "/home/sforesti/ros/home/ros/Repos/NIPS2016/ros/nips2016/logs/experiment_FGB_4.pickle"

with open(filename, 'r') as f:
    log = cPickle.load(f)
f.close()

#n = len(log["sm_data"]["mod2"][1])
n=1500

#print log["sm_data"]["mod2"][1]

j_touch = 0
b_touch = 0
fig_n = 1
for i in range(n):
    if np.linalg.norm(log["sm_data"]["mod2"][1][i] - np.array([-1.,  0., -1.,  0., -1.,  0., -1.,  0., -1.,  0., -1.,  0., -1., 0., -1.,  0., -1.,  0., -1.,  0.])) > 0.01:
        j_touch += 1
        if abs(list(log["sm_data"]["mod5"][1][i])[2:][0] -  list(log["sm_data"]["mod5"][1][i])[2:][-2]) > 0.05:
            print fig_n
            print "ergo angle", list(log["sm_data"]["mod4"][1][i])[1:][::2]
            print "ball angle", list(log["sm_data"]["mod5"][1][i])[2:][::2]
            b_touch += 1

            fig, ax = plt.subplots()
            plt.plot(np.cos(np.pi * np.array(log["sm_data"]["mod4"][1][i])[1:][::2]), np.sin(np.pi * np.array(log["sm_data"]["mod4"][1][i])[1:][::2]), "r")
            plt.plot(np.cos(np.pi * np.array(log["sm_data"]["mod5"][1][i])[2:][::2]), np.sin(np.pi * np.array(log["sm_data"]["mod5"][1][i])[2:][::2]), "b")
            for j in range(len(np.array(log["sm_data"]["mod5"][1][i])[2:][::2])):
                plt.plot([np.cos(np.pi * np.array(log["sm_data"]["mod5"][1][i])[2:][::2][j]), np.cos(np.pi * np.array(log["sm_data"]["mod4"][1][i])[1:][::2][j])], 
                         [np.sin(np.pi * np.array(log["sm_data"]["mod5"][1][i])[2:][::2][j]), np.sin(np.pi * np.array(log["sm_data"]["mod4"][1][i])[1:][::2][j])], "k")
                
            plt.xlim([-1, 1])
            plt.ylim([-1, 1])
            plt.show(block=False)
            fig_n += 1
plt.show()