import cPickle
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import sys

log_file = '/home/sforesti/scm/Flowers/NIPS2016/data/2016-11-30_11-17-25_v5_with_hand.pickle'


import brewer2mpl
bmap = brewer2mpl.get_map('Dark2', 'qualitative', 7)
colors = bmap.mpl_colors


with open(log_file, 'r') as f:
    data = cPickle.load(f)
    f.close()



s_data = {}
s_data["s_hand"] = np.array(data["sm_data"]["mod1"][1])[:,[15, 16, 17, 27, 28, 29]]
s_data["s_joystick_1"] = np.array(data["sm_data"]["mod2"][1])[:,[5, 6, 10, 11, 18, 19]]
s_data["s_joystick_2"] = np.array(data["sm_data"]["mod3"][1])[:,[5, 6, 10, 11, 18, 19]]
s_data["s_ergo"] = np.array(data["sm_data"]["mod4"][1])[:,[1+5, 1+6, 1+10, 1+11, 1+18, 1+19]]
s_data["s_ball"] = np.array(data["sm_data"]["mod5"][1])[:,[2+5, 2+6, 2+10, 2+11, 2+18, 2+19]]
s_data["s_light"] = np.array(data["sm_data"]["mod6"][1])[:,[2, 4, 6, 8, 10, 11]]
s_data["s_sound"] = np.array(data["sm_data"]["mod7"][1])[:,[2, 4, 6, 8, 10, 11]]





sw = 20
n = 5000
p = 100
gs = 5


x = np.array(np.linspace(0,n,n/p+1), dtype=int)



mins = np.array([-1., -1., -1., -1., -1., -1.])
maxs = np.array([1., 1., 1., 1., 1., 1.])





def runningMeanFast(x, sw):
    return np.convolve(x, np.ones((sw,))/sw, mode="valid")



def compute_explo(data, mins, maxs, checkpoints=None):
    if checkpoints is None:
        checkpoints = [len(data)]
    n = len(mins)
    assert len(data[0]) == n
    epss = (maxs - mins) / gs
    grid = np.zeros([gs] * n)
    #print np.size(grid), mins, maxs
    res = [0]
    for c in range(1, len(checkpoints)):
        for i in range(checkpoints[c-1], checkpoints[c]):
            idxs = np.array((data[i] - mins) / epss, dtype=int)
            #print c, i, idxs
            idxs[idxs>=gs] = gs-1
            idxs[idxs<0] = 0
            #print idxs
            grid[tuple(idxs)] = grid[tuple(idxs)] + 1
        grid[grid > 1] = 1
        res.append(np.sum(grid))
    return np.array(res)


space_names = dict(s_hand="Hand",
                   s_joystick_1="Joystick_1",
                   s_joystick_2="Joystick_2",
                   s_ergo="Ergo",
                   s_ball="Ball",
                   s_light="Light",
                   s_sound="Sound")



fig, ax = plt.subplots()

i = 0
for space in ["s_hand", "s_joystick_1", "s_joystick_2", "s_ergo", "s_ball", "s_light", "s_sound"]:
    explo = compute_explo(s_data[space], mins, maxs, x)
    ax.plot(x, explo, label=space_names[space], lw=4, color=colors[i])
    i += 1
    
plt.legend(loc="top_left", fontsize=16)
plt.ylabel("Explored cells", fontsize=20)
plt.xlabel("Trials", fontsize=20)
    
plt.savefig("exploration.pdf", format='pdf', bbox_inches='tight')
plt.show()
    
    
    