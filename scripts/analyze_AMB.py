import os
import sys
import cPickle
import numpy as np
import matplotlib.pyplot as plt


# PARAMS
plt.rc('text', usetex=True)
plt.rc('font', family='serif')


iterations = 3000

    
data = {}
for i in range(5):
        
    filename = "/home/sforesti/ros/home/ros/Repos/NIPS2016/ros/nips2016/logs/experiment_AMB_" + str(i) + ".pickle"
    
    with open(filename, 'r') as f:
        log = cPickle.load(f)
    f.close()
    data[i] = log
    
    print i, len(log["sm_data"]["mod2"][1])


labels = dict(mod1="Hand", 
             mod2="Joystick_1", 
             mod3="Joystick_2", 
             mod4="Ergo", 
             mod5="Ball", 
             mod6="Light", 
             mod7="Sound")

for i in range(5):
    plt.figure()
    for mid in data[i]["interests_evolution"].keys():
        plt.plot(data[i]["interests_evolution"][mid][:iterations], lw=2, label=labels[mid])
    
    
    legend = plt.legend(frameon=True, fontsize=18, loc="left")
    plt.xticks(fontsize = 16)
    plt.yticks(fontsize = 16)
    plt.xlabel("Iterations", fontsize=18)
    plt.ylabel("Interest", fontsize=18)
    frame = legend.get_frame()
    frame.set_facecolor('1.')
    frame.set_edgecolor('0.')
        
        
    plt.savefig("/home/sforesti/scm/Flowers/NIPS2016/data/figs/interest_AMB_" + str(i) + '.pdf', format='pdf', dpi=100, bbox_inches='tight')
    plt.show(block=False)
plt.show()