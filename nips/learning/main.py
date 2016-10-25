import sys
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('../../')
from nips.environment.environment import environment
from nips.learning.supervisor import Supervisor




class Learning(object):
    def __init__(self, environment):
        self.environment = environment
        self.agent = Supervisor(self.environment)
        
        
    def produce(self, space=None):
        if space is None:
            # Autonomous step
            context = environment.get_current_context()
            return self.agent.produce(context)
        else:
            # Force space
            assert space in ["s_hand", "s_joystick", 's_ergo', "s_ball", "s_light", "s_sound"]
            return self.agent.produce(context, space=space)
            
            
    def perceive(self, s, m_traj=None):
        if m_traj is None:
            # Perception of environment when m was produced
            self.agent.perceive(s)
        else:
            # Demonstration of a torso arm trajectory
            m = self.torsodemo2m(m_traj)
            self.agent.perceive(s, m=m)
        
        
    def torsodemo2m(self, m_traj):
        # m_traj is the trajectory of the 4 motors: must be shaped 25*4
        return self.environment.motor_dmp.imitate(m_traj)
                

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(np.array(self.agent.interests_evolution[:1000]), lw=2)
        ax.legend(["s_hand", "s_joystick", "s_ergo", "s_ball", "s_light", "s_sound"], ncol=3)
        ax.set_xlabel('Time steps', fontsize=20)
        ax.set_ylabel('Learning progress', fontsize=20)
        plt.show(block=True)
        
        
        
if __name__ == "__main__":
    
    learning = Learning(environment)
    for i in range(10000):
        m = learning.produce()
        s = environment.update(m)
        learning.perceive(s)
    learning.plot()