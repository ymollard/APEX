import sys
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('../../')
from nips.environment.environment import TestEnvironment
from nips.learning.supervisor import Supervisor




class Learning(object):
    def __init__(self, environment):
        self.environment = environment
        self.agent = Supervisor(self.environment)
        
        
    def produce(self, context, space=None):
        # context is the rotation of the ergo and the ball: "context = environment.get_current_context()"
        if space is None:
            # Autonomous step
            return self.agent.produce(context)
        else:
            # Force space
            assert space in ["s_hand", "s_joystick", 's_ergo', "s_ball", "s_light", "s_sound"]
            return self.agent.produce(context, space=space)
            
            
    def perceive(self, s, m_demo=None, j_demo=False):
        if m_demo is not None:
            # Demonstration of a torso arm trajectory converted to weights with "m_demo=environment.torsodemo2m(m_traj)"
            self.agent.perceive(s, m_demo=m_demo)
        elif j_demo:
            self.agent.perceive(s, j_demo=True)
        else:
            # Perception of environment when m was produced
            self.agent.perceive(s)
                

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(np.array(self.agent.interests_evolution[:1000]), lw=2)
        ax.legend(["s_hand", "s_joystick", "s_ergo", "s_ball", "s_light", "s_sound"], ncol=3)
        ax.set_xlabel('Time steps', fontsize=20)
        ax.set_ylabel('Learning progress', fontsize=20)
        plt.show(block=True)
        
        
        
if __name__ == "__main__":
    environment = TestEnvironment()
    learning = Learning(environment)
    for i in range(10000):
        context = environment.get_current_context()
        m = learning.produce(context)
        s = environment.update(m)
        learning.perceive(s)
    learning.plot()