
import os
import pickle

import matplotlib.pyplot as plt

log_dir = "../data/logs/"
name = "motor_babbling"

filename = os.path.join(log_dir, name + ".pickle")
with open(filename, 'r') as f:
    data = pickle.load(f)
    
print "\nsm_data", data["sm_data"]
print "\nim_data", data["im_data"]
print "\nchosen_modules", data["chosen_modules"]
print "\nprogresses_evolution", data["progresses_evolution"]
print "\ninterests_evolution", data["interests_evolution"]
print "\nnormalized_interests_evolution", data["normalized_interests_evolution"]



print data["normalized_interests_evolution"]
fig, ax = plt.subplots()
ax.plot(data["normalized_interests_evolution"], lw=2)
ax.legend(["Hand", "Joystick_1", "Joystick_2", "Ergo", "Ball", "Light", "Sound"], ncol=3)
ax.set_xlabel('Time steps', fontsize=20)
ax.set_ylabel('Learning progress', fontsize=20)
plt.show(block=True)


fig, ax = plt.subplots()
plt.title("Hand")
for s in data["sm_data"]["mod1"][1]:
    print s
    x = s[range(0, 30, 3)]
    y = s[range(1, 30, 3)]
    z = s[range(2, 30, 3)]
    ax.plot(x, color="r")
    ax.plot(y, color="g")
    ax.plot(z, color="b")
plt.legend(["x", "y", "z"])
ax.set_xlabel('Time steps', fontsize=20)
plt.legend()
plt.ylim([-1.1, 1.1])



fig, ax = plt.subplots()
plt.title("Joystick 1")
for s in data["sm_data"]["mod2"][1]:
    x = s[range(0, 20, 2)]
    y = s[range(1, 20, 2)]
    ax.plot(x, color="r")
    ax.plot(y, color="g")
plt.legend(["Forward", "LR"])
ax.set_xlabel('Time steps', fontsize=20)
plt.legend()
plt.ylim([-1.1, 1.1])



fig, ax = plt.subplots()
plt.title("Joystick 2")
for s in data["sm_data"]["mod3"][1]:
    x = s[range(0, 20, 2)]
    y = s[range(1, 20, 2)]
    ax.plot(x, color="r")
    ax.plot(y, color="g")
plt.legend(["LR", "Forward"])
ax.set_xlabel('Time steps', fontsize=20)
plt.legend()
plt.ylim([-1.1, 1.1])



fig, ax = plt.subplots()
plt.title("Ergo")
for s in data["sm_data"]["mod4"][1]:
    x = s[range(0, 20, 2)]
    y = s[range(1, 20, 2)]
    ax.plot(x, color="r")
    ax.plot(y, color="g")
plt.legend(["Angle", "Elongation"])
ax.set_xlabel('Time steps', fontsize=20)
plt.legend()
plt.ylim([-1.1, 1.1])



fig, ax = plt.subplots()
plt.title("Ball")
for s in data["sm_data"]["mod5"][1]:
    print "s ball", s
    x = s[range(0, 20, 2)]
    y = s[range(1, 20, 2)]
    ax.plot(x, color="r")
    ax.plot(y, color="g")
plt.legend(["Angle", "Elongation"])
ax.set_xlabel('Time steps', fontsize=20)
plt.legend()
plt.ylim([-1.1, 1.1])



fig, ax = plt.subplots()
plt.title("Light")
for s in data["sm_data"]["mod6"][1]:
    ax.plot(s, color="r")
ax.set_xlabel('Time steps', fontsize=20)
plt.ylim([-1.1, 1.1])



fig, ax = plt.subplots()
plt.title("Sound")
for s in data["sm_data"]["mod7"][1]:
    ax.plot(s, color="r")
ax.set_xlabel('Time steps', fontsize=20)
plt.ylim([-1.1, 1.1])




plt.show(block=True)
