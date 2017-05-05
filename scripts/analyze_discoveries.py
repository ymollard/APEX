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
      "Joystick_L": bmap.mpl_colors[2], 
      "Joystick_R":bmap.mpl_colors[3], 
      "Ergo":bmap.mpl_colors[4], 
      "Ball":bmap.mpl_colors[5], 
      "Light":bmap.mpl_colors[6], 
      "Sound":bmap.mpl_colors[7]
      }

# PARAMS
path = "/home/sforesti/scm/Flowers/NIPS2017/data/logs/"
experiment_name = "seb_holidays"
configs = dict(RMB=3, AMB=3)
#configs = dict(RmB=1, AMB=5)


n_logs = 1
n = 5000 # Iterations taken into account
p = 20 # Number of checkpoints
x = range(n)



def discovery(data):
    n = len(data)
    result = [0.]
    for i in range(1, n):
        x = data[i]
        if np.linalg.norm(np.array(x)[::2] - np.mean(np.array(x)[::2])) < 0.01 and np.linalg.norm(np.array(x)[1::2] - np.mean(np.array(x)[1::2])) < 0.01:
            result.append(0.)
        else:
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
              
    
s_spaces = ["Hand", "Joystick_L", "Joystick_R", "Ergo", "Ball", "Light", "Sound"]


if True:

    explo = {}
    explo_gain = {}
    explo_discovery = {}
    bootstrapped_s = {}
    
    
    for s_space in s_spaces:
        explo[s_space] = {}
        explo_gain[s_space] = {}
        bootstrapped_s[s_space] = {}
    
    

    for config in configs.keys():
        
    
        for s_space in s_spaces:
            explo[s_space][config] = {}
            explo_gain[s_space][config] = {}
            bootstrapped_s[s_space][config] = {}
    
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
                    "Joystick_L":"mod3",
                    "Joystick_R":"mod2",
                    "Ergo":"mod4",
                    "Ball":"mod5",
                    "Light":"mod6",
                    "Sound":"mod7"}
            
            cdims = dict(Hand=0,
                        Joystick_L=0,
                        Joystick_R=0,
                        Ergo=1,
                        Ball=2,
                        Light=2,
                        Sound=2)
            
            
            for s_space in explo.keys():
                #print "Analysis", s_space
                
                explo_gain[s_space][config][trial] = discovery([log["sm_data"][dims[s_space]][1][i][cdims[s_space]:] for i in range(n)])
                bootstrapped_s[s_space][config][trial] = next((i for i, x in enumerate(explo_gain[s_space][config][trial]) if x), n)
                #print s_space, explo[s_space][config][trial][:20]
                
            bootstrapped_s["motor_babbling"] = {}
            bootstrapped_s["motor_babbling"][config] = {}
            bootstrapped_s["motor_babbling"][config][trial] = 0
            
            for s_space1 in explo.keys()+ ["motor_babbling"]:
                explo_discovery[config][trial][s_space1] = {}
                for s_space2 in explo.keys():
                    explo_discovery[config][trial][s_space1][s_space2] = np.zeros(p)
                    
            for i in range(1,n-1):
                if i in range(1200, 2000):
                    print i, "\nBall:", log["sm_data"]["mod5"][1][i]
                for s_space1 in explo_discovery[config][trial].keys():
                    mid = dims[s_space1]
                    if mid == log["chosen_modules"][i]:
                        for s_space2 in explo.keys():
                            #print
                            g = explo_gain[s_space2][config][trial][i]
                            if g > 0 and s_space2 in ["Ball"]:
                                m = log["sm_data"]["mod1"][0][i]
                                #print "Iteration:", i, ", Chosen space:", s_space1, ", Gain in", s_space2, ":", g, "bootstrapped_s:", bootstrapped_s[s_space1][config][trial]
#                                 print i, "\nJR:", log["sm_data"]["mod2"][1][i]
#                                 print i, "\nJL:", log["sm_data"]["mod3"][1][i]
#                                 print i, "\nErgo:", log["sm_data"]["mod4"][1][i]
#                                 print i, "\nBall:", log["sm_data"]["mod5"][1][i]
                            if g > 0 and bootstrapped_s[s_space1][config][trial] < i:
                                explo_discovery[config][trial][s_space1][s_space2][i/(n/p)] += g
                    
                    
            for s_space1 in explo_discovery[config][trial].keys():
                #print trial, s_space1, [log["chosen_modules"][i*(n/10):(i+1)*(n/10)].count(dims[s_space1]) for i in range(10)]
                for s_space2 in explo.keys():
                    explo_discovery[config][trial][s_space1][s_space2] /= [log["chosen_modules"][i*(n/p):(i+1)*(n/p)].count(dims[s_space1]) for i in range(p)]
#                     if s_space2 == "Hand":
#                         print "   ", s_space2, explo_discovery[config][trial][s_space1][s_space2]
                    
                    
    with open(path + 'analysis_explo_discovery.pickle', 'wb') as f:
        cPickle.dump(explo_discovery, f)

else:
    
    with open(path + 'analysis_explo_discovery.pickle', 'r') as f:
        explo_discovery = cPickle.load(f)
    f.close()
    
    config = "RMB"
    trials = range(configs[config])
    trials = [0]
    print trials
    
    labels = {
              "motor_babbling":"$Random$", 
              "Hand": "$Hand$", 
              "Joystick_L": "$Joystick_L$", 
              "Joystick_R":"$Joystick_R$", 
              "Ergo":"$Ergo$", 
              "Ball":"$Ball$", 
              "Light":"$Light$", 
              "Sound":"$Sound$"
              }
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    for s_space2 in ["Hand", "Joystick_L", "Joystick_R", "Ergo", "Ball"]:#, "Light", "Sound"]:
            
                
        fig, ax = plt.subplots()
        fig.canvas.set_window_title(s_space2)
        plt.title("Discoveries in $"+s_space2+"$ space, while exploring...", fontsize=24)

        total_explo_s_space2 = np.sum([[explo_discovery[config][trial][s_space1][s_space2] for trial in trials] for s_space1 in s_spaces])
            
        for s_space1 in ["motor_babbling", "Hand", "Joystick_L", "Joystick_R", "Ergo", "Ball"]:#, "Light", "Sound"]:
#             if s_space1 == "Hand" and s_space2 == "Ball":
#                 print s_space1, s_space2, [explo_discovery[config][trial][s_space1][s_space2] for trial in range(configs[config])]
            plt.plot(np.linspace(n/p, n, p), 100. * np.sum([explo_discovery[config][trial][s_space1][s_space2] for trial in trials], axis=0) / total_explo_s_space2, lw=3, color=colors[s_space1], label=labels[s_space1])
            
                
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
            
        