import os
import sys
import cPickle
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

import brewer2mpl
bmap = brewer2mpl.get_map('Dark2', 'qualitative', 4)
colors = bmap.mpl_colors

# PARAMS
path = "/home/sforesti/scm/Flowers/NIPS2016/data/logs/"
configs = dict(RMB=5)
#configs = dict(RmB=1, AMB=5)

config_colors = dict(AMB=colors[0], RmB=colors[1], FGB=colors[2], RMB=colors[3])


sw = 20
n_logs = 1
n = 3000
p = 100
x = np.array(np.linspace(0,n,n/p+1), dtype=int)

gss = [0, 1000, 100, 20, 10, 6, 5, 4, 3, 3]


def compute_explo(data, min_, max_, checkpoints=None):
    if checkpoints is None:
        checkpoints = [len(data)]
    nd = len(data[0]) / 10

    gs = gss[nd]
    epss = [(max_ - min_) / gs] * nd
    #print gs, nd, epss
    grid = np.zeros([gs] * nd)

    res = [0]
    for c in range(1, len(checkpoints)):
        for i in range(checkpoints[c-1], checkpoints[c]):
            for timepoint in range(10):
                s = np.array(data[i])[nd*timepoint:nd*(timepoint+1)]
                idxs = np.array((s - min_) / epss, dtype=int)
                #print c, i, idxs
                idxs[idxs>=gs] = gs-1
                idxs[idxs<0] = 0
                #print idxs
                grid[tuple(idxs)] = grid[tuple(idxs)] + 1
        grid[grid > 1] = 1
        res.append(np.sum(grid))
    return np.array(res) / gs ** nd
    

if True:

    
    explo = {}
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
    
        for trial in range(configs[config]):
            
            print "\nLoading", config, trial
            
            filename = path + "discovery_" + config + "_" + str(trial) + ".pickle"
            with open(filename, 'r') as f:
                log = cPickle.load(f)
            f.close()
            
            
            if not (config == "FGB"):
                j_touch = 0
                b_touch = 0
                
                for i in range(n):
                    if i < 100:
                        #print i, "Hand:", log["sm_data"]["mod1"][1][i]
                        #print i, "Ergo:", log["sm_data"]["mod4"][1][i]
                        print i, "Ball:", log["sm_data"]["mod5"][1][i][[2+4, 2+5, 2+8, 2+9, 2+12, 2+13, 2+16, 2+17]]
                        #print i, "Light:", log["sm_data"]["mod6"][1][i]
                    if np.linalg.norm(log["sm_data"]["mod2"][1][i] - np.array([-1.,  0., -1.,  0., -1.,  0., -1.,  0., -1.,  0., -1.,  0., -1., 0., -1.,  0., -1.,  0., -1.,  0.])) > 0.01:
                        j_touch += 1
                        #print "joystick", list(log["sm_data"]["mod2"][1][i])
                        if abs(list(log["sm_data"]["mod5"][1][i])[2:][0] -  list(log["sm_data"]["mod5"][1][i])[2:][-2]) > 0.05:
                            #print "ball", list(log["sm_data"]["mod5"][1][i])[2:]
                            b_touch += 1
                print
                print "Motor Babbling Iterations:", n
                print "Joystick touched:", j_touch, "percentage:", int(100. * j_touch / n), "%"
                print "Ball touched:", b_touch, "percentage:", int(100. * b_touch / n), "%"
                print
                
            
            if config == "FGB":
            
                dims = dict(Hand=range(30),
                            Joystick_1=range(30, 50),
                            Joystick_2=range(50, 70),
                            Ergo=range(70, 90),
                            Ball=range(90, 110),
                            Light=range(110, 120),
                            Sound=range(120, 130))
                
                
                for s_space in dims.keys():
                    explo[s_space][config][trial] = compute_explo([log["sm_data"]["mod"][1][i][2:][dims[s_space]] for i in range(n)], -1., 1., x)
                    
            else:
                        
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
                    print "Analysis", s_space
                    
                    explo[s_space][config][trial] = compute_explo([log["sm_data"][dims[s_space]][1][i][cdims[s_space]:] for i in range(n)], -1., 1., x)
                    print explo[s_space][config][trial]
                 
                 
    with open(path + 'analysis_explo.pickle', 'wb') as f:
        cPickle.dump(explo, f)

else:
    
    with open(path + 'analysis_explo.pickle', 'r') as f:
        explo = cPickle.load(f)
    f.close()
    
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    for s_space in explo.keys():
                 
        fig, ax = plt.subplots()
        fig.canvas.set_window_title(s_space)
        plt.title('Exploration of $' + s_space + "$ space", fontsize=24)
        
        for config in configs.keys():
            ys = [100.*explo[s_space][config][trial] for trial in range(configs[config])] 
            ymed = np.percentile(ys, 50, axis=0)
            ymax = np.percentile(ys, 100, axis=0)
            ymin = np.percentile(ys, 0, axis=0)
            
            plt.plot(x, ymed, lw=2, color=config_colors[config], label=config)
            plt.fill_between(x, ymin, ymax, color=config_colors[config], alpha=0.2)
        
            
        legend = plt.legend(frameon=True, fontsize=18)
        plt.xticks(fontsize = 16)
        plt.yticks(fontsize = 16)
        plt.xlabel("Iterations", fontsize=18)
        plt.ylabel("Exploration \%", fontsize=18)
        frame = legend.get_frame()
        frame.set_facecolor('1.')
        frame.set_edgecolor('0.')
        
        #plt.savefig("/home/sforesti/scm/PhD/cogsci2016/include/obj-explo.pdf", format='pdf', dpi=100, bbox_inches='tight')
        plt.savefig("/home/sforesti/scm/Flowers/NIPS2016/data/figs/explo_" + s_space + '.pdf', format='pdf', dpi=100, bbox_inches='tight')
        
        
        plt.show(block=False)
    plt.show()
            
        
print explo
