import os
import sys
import cPickle
import numpy as np
import matplotlib.pyplot as plt


# PARAMS
plt.rc('text', usetex=True)
plt.rc('font', family='serif')


simu = False

if simu:
    
    # From SIMU
    path = "/home/sforesti/catkin_ws/src/nips2017/logs/"
    experiment_name = "experiment"
    configs = dict(AMB=10)
    n = 10000
    j_error = 0.1
else:
    
    # PARAMS
    experiment_name = "postholidays"
    path = "/data/APEX/" + experiment_name + "/"
    configs = dict(AMB=[0, 1, 4])
    configs = dict(AMB=[0, 1, 3, 4])
    n = 20000
    j_error = 0.3



config = "AMB"
    
data = {}
for i in configs[config]:
        
    filename = path + experiment_name + "_" + config + "_" + str(i) + ".pickle"
    
    with open(filename, 'r') as f:
        log = cPickle.load(f)
    f.close()
    data[i] = log
    
    print i, len(log["sm_data"]["mod2"][1])
    #print len(data[i]["interests_evolution"])
    #print data[i]["interests_evolution"]


labels = dict(mod1="Hand", 
             mod2="Joystick Right", 
             mod3="Joystick Left", 
             mod4="Ergo", 
             mod5="Ball", 
             mod6="Light", 
             mod7="Sound",
             mod8="HR",
             mod9="BA",
             mod10="AR",
             mod11="O1",
             mod12="O2",
             mod13="O3",
             mod14="R1",
             mod15="R2",
             )

for i in configs["AMB"]:
    plt.figure()
    for mid in ["mod1", "mod3", "mod2", "mod4", "mod5", "mod6", "mod7", "mod14", "mod15"]:
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
