import sys
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('../../')
from nips.environment.environment import environment
from nips.learning.supervisor import Supervisor

iterations = 10000

agent = Supervisor(environment)


for iteration in range(iterations):
    context = environment.get_current_context()
    m = agent.produce(context)
    s = environment.update(m)
    agent.perceive(s)
    


fig, ax = plt.subplots()
ax.plot(np.array(agent.interests_evolution[:1000]), lw=2)
ax.legend(["s_hand", "s_joystick", "s_ergo", "s_ball", "s_light", "s_sound"], ncol=3)
ax.set_xlabel('Time steps', fontsize=20)
ax.set_ylabel('Learning progress', fontsize=20)
plt.show(block=True)