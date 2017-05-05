import os
import sys
import cPickle
import numpy as np
import matplotlib.pyplot as plt
#import seaborn as sns

import brewer2mpl
bmap = brewer2mpl.get_map('Dark2', 'qualitative', 7)
colors = bmap.mpl_colors

# PARAMS
path = "/home/sforesti/scm/Flowers/NIPS2016/data/logs/"
configs = dict(RMB=5)
#configs = dict(RmB=1, AMB=5)



sw = 20
n_logs = 1
n = 3000
x = range(n)

gss = [0, 1000, 100, 20, 10, 6, 5, 4, 4, 3]


def compute_explo(data, min_, max_, checkpoints=None):
    if checkpoints is None:
        checkpoints = [len(data)]
    
    if len(data[0]) == 20:
        nd = 8
        idx = [4, 5, 8, 9, 12, 13, 16, 17]
    else:
        nd = 4
        idx = [2, 4, 6, 8]

    gs = gss[nd]
    epss = [(max_ - min_) / gs] * nd
    #print gs, nd, epss
    grid = np.zeros([gs] * nd)

    res = [0]
    for c in range(1, len(checkpoints)):
        for i in range(checkpoints[c-1], checkpoints[c]):
            s = np.array(data[i])[idx]
            idxs = np.array((s - min_) / epss, dtype=int)
            #print c, i, idxs
            idxs[idxs>=gs] = gs-1
            idxs[idxs<0] = 0
            #print idxs
            grid[tuple(idxs)] = grid[tuple(idxs)] + 1
        grid[grid > 1] = 1
        res.append(np.sum(grid))
    return np.array(res) / gs ** nd
    

if False:

    
    explo = {}
    explo_gain = {}
    
    explo['Hand'] = {}
    explo['Joystick_1'] = {}
    explo['Joystick_2'] = {}
    explo['Ergo'] = {}
    explo['Ball'] = {}
    explo['Light'] = {}
    explo['Sound'] = {}
    

    for config in configs.keys():
        
    
        explo['Hand'][config] = {}
        explo['Joystick_1'][config] = {}
        explo['Joystick_2'][config] = {}
        explo['Ergo'][config] = {}
        explo['Ball'][config] = {}
        explo['Light'][config] = {}
        explo['Sound'][config] = {}
    
        explo_gain[config] = {}


        for trial in range(configs[config]):
            
            explo_gain[config][trial] = {}
            
            print "\nLoading", config, trial
            
            filename = path + "experiment_" + config + "_" + str(trial) + ".pickle"
            with open(filename, 'r') as f:
                log = cPickle.load(f)
            f.close()
            
            print log["chosen_modules"][:20]
            
            dims = dict(Hand="mod1",
                        Joystick_1="mod2",
                        Joystick_2="mod3",
                        Ergo="mod4",
                        Ball="mod5",
                        Light="mod6",
                        Sound="mod7")
            
            cdims = dict(Hand=0,
                        Joystick_1=0,
                        Joystick_2=0,
                        Ergo=1,
                        Ball=2,
                        Light=2,
                        Sound=2)
            
            
            for s_space in explo.keys():
                #print "Analysis", s_space
                
                explo[s_space][config][trial] = compute_explo([log["sm_data"][dims[s_space]][1][i][cdims[s_space]:] for i in range(n)], -1., 1., x)
                #print s_space, explo[s_space][config][trial][:20]
                
            for s_space1 in explo.keys():
                explo_gain[config][trial][s_space1] = {}
                for s_space2 in explo.keys():
                    mid = dims[s_space1]
                    explo_gain[config][trial][s_space1][s_space2] = np.zeros(10)
                    #print
                    for i in range(1,3000-1):
                        if mid == log["chosen_modules"][i]:
                            explo_gain[config][trial][s_space1][s_space2][i/300] += explo[s_space2][config][trial][i+1] - explo[s_space2][config][trial][i]
                            if s_space1 == "Hand" and s_space2 == "Ball":
                                print trial, i, explo[s_space2][config][trial][i], explo[s_space2][config][trial][i+1], explo[s_space2][config][trial][i+1] - explo[s_space2][config][trial][i]
                            #if i < 20:
                                #print s_space1, s_space2, i, explo[s_space2][config][trial][i+1] - explo[s_space2][config][trial][i]
                    
                    
        for trial in range(configs[config]):
            for s_space1 in explo.keys():
                for s_space2 in explo.keys():
                    explo_gain[config][trial][s_space1][s_space2] /= np.max([explo[s_space2][config][trial_][-1] for trial_ in range(configs[config])])
                    
                    
                    
    with open(path + 'analysis_explo_gain.pickle', 'wb') as f:
        cPickle.dump(explo_gain, f)

else:
    
    with open(path + 'analysis_explo_gain.pickle', 'r') as f:
        explo_gain = cPickle.load(f)
    f.close()
    
    config = "RMB"
    print range(configs[config])
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    for s_space1 in explo_gain[config][0].keys():
            
                
        fig, ax = plt.subplots()
        fig.canvas.set_window_title(s_space1)
        plt.title("$"+s_space1+"$", fontsize=24)
            
        for s_space2 in explo_gain[config][0][s_space1].keys():
            if s_space1 == "Hand" and s_space2 == "Ball":
                print s_space1, s_space2, [explo_gain[config][trial][s_space1][s_space2] for trial in range(configs[config])]
            plt.plot(np.linspace(300, 3000, 10), np.mean([explo_gain[config][trial][s_space1][s_space2] for trial in range(configs[config])], axis=0), lw=2, label="$"+s_space2+"$")
            
                
        legend = plt.legend(frameon=True, fontsize=18)
        plt.xticks(fontsize = 16)
        plt.yticks(fontsize = 16)
        plt.xlabel("Iterations", fontsize=18)
        plt.ylabel("Exploration Gain \%", fontsize=18)
        frame = legend.get_frame()
        frame.set_facecolor('1.')
        frame.set_edgecolor('0.')
        
        #plt.savefig("/home/sforesti/scm/PhD/cogsci2016/include/obj-explo.pdf", format='pdf', dpi=100, bbox_inches='tight')
        plt.savefig("/home/sforesti/scm/Flowers/NIPS2016/data/figs/explo_gain_" + s_space1 + '.pdf', format='pdf', dpi=100, bbox_inches='tight')
        
        
        plt.show(block=False)
    plt.show()
            
        