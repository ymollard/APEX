import os
import sys
import cPickle
import numpy as np
import matplotlib.pyplot as plt
#import seaborn as sns

import brewer2mpl
bmap = brewer2mpl.get_map('Dark2', 'qualitative', 8)
#colors = bmap.mpl_colors
colors = {
      "motor_babbling":bmap.mpl_colors[0], 
      "Hand": bmap.mpl_colors[1], 
      "Joystick_1": bmap.mpl_colors[2], 
      "Joystick_2":bmap.mpl_colors[3], 
      "Ergo":bmap.mpl_colors[4], 
      "Ball":bmap.mpl_colors[5], 
      "Light":bmap.mpl_colors[6], 
      "Sound":bmap.mpl_colors[7]
      }

# PARAMS
path = "/home/sforesti/scm/Flowers/NIPS2017/data/logs/"
experiment_name = "discovery"
configs = dict(RMB=5)
#configs = dict(RmB=1, AMB=5)


sw = 20
n_logs = 1
n = 500
x = range(n)

gss = [0, 1000, 100, 20, 10, 6, 5, 4, 3, 3]


def discovery(data):
    n = len(data)
    result = [0.]
    for i in range(1, n):
        x = data[i]
        min_dist = np.inf
        for j in range(i):
            y = data[j]
            min_dist = min(min_dist, np.linalg.norm(np.array(x) - np.array(y)))
        result.append(min_dist)    
    return result

def dist(x, grid):
    try:
        return min([np.linalg.norm(idx - x) for idx in np.argwhere(grid)])
    except ValueError:
        return 0.
              
    

if False:

    
    explo = {}
    explo_gain = {}
    explo_discovery = {}
    
    explo['Hand'] = {}
    explo['Joystick_1'] = {}
    explo['Joystick_2'] = {}
    explo['Ergo'] = {}
    explo['Ball'] = {}
    explo['Light'] = {}
    explo['Sound'] = {}
    
    explo_gain['Hand'] = {}
    explo_gain['Joystick_1'] = {}
    explo_gain['Joystick_2'] = {}
    explo_gain['Ergo'] = {}
    explo_gain['Ball'] = {}
    explo_gain['Light'] = {}
    explo_gain['Sound'] = {}
    

    for config in configs.keys():
        
    
        explo['Hand'][config] = {}
        explo['Joystick_1'][config] = {}
        explo['Joystick_2'][config] = {}
        explo['Ergo'][config] = {}
        explo['Ball'][config] = {}
        explo['Light'][config] = {}
        explo['Sound'][config] = {}
        
        explo_gain['Hand'][config] = {}
        explo_gain['Joystick_1'][config] = {}
        explo_gain['Joystick_2'][config] = {}
        explo_gain['Ergo'][config] = {}
        explo_gain['Ball'][config] = {}
        explo_gain['Light'][config] = {}
        explo_gain['Sound'][config] = {}
    
        explo_discovery[config] = {}


        for trial in range(configs[config]):
            
            explo_discovery[config][trial] = {}
            
            print "\nLoading", config, trial
            
            filename = path + experiment_name + "_" + config + "_" + str(trial) + ".pickle"
            with open(filename, 'r') as f:
                log = cPickle.load(f)
            f.close()
            
            #print log["chosen_modules"][:50]
            
            dims = {"motor_babbling":"motor_babbling",
                    "Hand":"mod1",
                        "Joystick_1":"mod2",
                        "Joystick_2":"mod3",
                        "Ergo":"mod4",
                        "Ball":"mod5",
                        "Light":"mod6",
                        "Sound":"mod7"}
            
            cdims = dict(Hand=0,
                        Joystick_1=0,
                        Joystick_2=0,
                        Ergo=1,
                        Ball=2,
                        Light=2,
                        Sound=2)
            
            
            for s_space in explo.keys():
                #print "Analysis", s_space
                
                explo_gain[s_space][config][trial] = discovery([log["sm_data"][dims[s_space]][1][i][cdims[s_space]:] for i in range(n)])
                #print s_space, explo[s_space][config][trial][:20]
                
            for s_space1 in explo.keys()+ ["motor_babbling"]:
                explo_discovery[config][trial][s_space1] = {}
                for s_space2 in explo.keys():
                    explo_discovery[config][trial][s_space1][s_space2] = np.zeros(10)
                    
            for i in range(1,n-1):
                for s_space1 in explo_discovery[config][trial].keys():
                    mid = dims[s_space1]
                    if mid == log["chosen_modules"][i]:
                        for s_space2 in explo.keys():
                            #print
                            g = explo_gain[s_space2][config][trial][i]
                            if g > 0.01:
                                m = log["sm_data"]["mod1"][0][i]
                                print "Iteration:", i, ", Chosen space:", s_space1, ", Gain in", s_space2, ":", g, m[:5]
                            if g > 0:
                                explo_discovery[config][trial][s_space1][s_space2][i/(n/10)] += g
                    
                    
            for s_space1 in explo_discovery[config][trial].keys():
                print trial, s_space1, [log["chosen_modules"][i*(n/10):(i+1)*(n/10)].count(dims[s_space1]) for i in range(10)]
                for s_space2 in explo.keys():
                    explo_discovery[config][trial][s_space1][s_space2] /= [log["chosen_modules"][i*(n/10):(i+1)*(n/10)].count(dims[s_space1]) for i in range(10)]
                    if s_space2 == "Hand":
                        print "   ", s_space2, explo_discovery[config][trial][s_space1][s_space2]
                    
                    
    with open(path + 'analysis_explo_discovery.pickle', 'wb') as f:
        cPickle.dump(explo_discovery, f)

else:
    
    with open(path + 'analysis_explo_discovery.pickle', 'r') as f:
        explo_discovery = cPickle.load(f)
    f.close()
    
    config = "RMB"
    print range(configs[config])
    
    labels = {
              "motor_babbling":"$Random$", 
              "Hand": "$Hand$", 
              "Joystick_1": "$Joystick_1$", 
              "Joystick_2":"$Joystick_2$", 
              "Ergo":"$Ergo$", 
              "Ball":"$Ball$", 
              "Light":"$Light$", 
              "Sound":"$Sound$"
              }
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    for s_space2 in ["Hand", "Joystick_1", "Joystick_2", "Ergo", "Ball"]:#, "Light", "Sound"]:
            
                
        fig, ax = plt.subplots()
        fig.canvas.set_window_title(s_space2)
        plt.title("Discoveries in $"+s_space2+"$ space, while exploring...", fontsize=24)

        total_explo_s_space2 = np.sum([[explo_discovery[config][trial][s_space1][s_space2] for trial in range(configs[config])] for s_space1 in ["motor_babbling", "Hand", "Joystick_1", "Joystick_2", "Ergo", "Ball", "Light", "Sound"]])
            
        for s_space1 in ["motor_babbling", "Hand", "Joystick_1", "Joystick_2", "Ergo", "Ball"]:#, "Light", "Sound"]:
#             if s_space1 == "Hand" and s_space2 == "Ball":
#                 print s_space1, s_space2, [explo_discovery[config][trial][s_space1][s_space2] for trial in range(configs[config])]
            plt.plot(np.linspace(n/10, n, 10), 100. * np.sum([explo_discovery[config][trial][s_space1][s_space2] for trial in range(configs[config])], axis=0) / total_explo_s_space2, lw=3, color=colors[s_space1], label=labels[s_space1])
            
                
        legend = plt.legend(frameon=True, fontsize=18)
        plt.xticks(fontsize = 16)
        plt.yticks(fontsize = 16)
        plt.xlabel("Iterations", fontsize=18)
        plt.ylabel("Exploration Gain \%", fontsize=18)
        frame = legend.get_frame()
        frame.set_facecolor('1.')
        frame.set_edgecolor('0.')
        
        #plt.savefig("/home/sforesti/scm/PhD/cogsci2016/include/obj-explo.pdf", format='pdf', dpi=100, bbox_inches='tight')
        plt.savefig("/home/sforesti/scm/Flowers/NIPS2017/figs/explo_discovery_" + s_space2 + '.pdf', format='pdf', dpi=100, bbox_inches='tight')
        
        
        plt.show(block=False)
    plt.show()
            
        