import os
import sys
import cPickle
import numpy as np
import matplotlib.pyplot as plt


# PARAMS
plt.rc('text', usetex=True)
plt.rc('font', family='serif')


simu = True

if simu:
    
    # From SIMU
    path = "/home/sforesti/catkin_ws/src/nips2017/logs/"
    experiment_name = "experiment"
    configs = dict(AMB=8)
    n = 10000
    j_error = 0.1
else:
    
    # PARAMS
    path = "/home/sforesti/scm/Flowers/NIPS2017/data/logs/"
    experiment_name = "nips_4_mai"
    configs = dict(AMB=2)
    n = 5000
    j_error = 0.02



    
data = {}
for i in range(configs["AMB"]):
        
    filename = path + experiment_name + "_AMB_" + str(i) + ".pickle"
    
    with open(filename, 'r') as f:
        log = cPickle.load(f)
    f.close()
    data[i] = log
    
    print i, len(log["sm_data"]["mod2"][1])


labels = dict(mod1="Hand", 
             mod2="Joystick Right", 
             mod3="Joystick Left", 
             mod4="Ergo", 
             mod5="Ball", 
             mod6="Light", 
             mod7="Sound")

for i in range(configs["AMB"]):
    plt.figure()
    for mid in data[i]["interests_evolution"].keys():
        plt.plot(data[i]["interests_evolution"][mid][:n], lw=2, label=labels[mid])
    
    
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