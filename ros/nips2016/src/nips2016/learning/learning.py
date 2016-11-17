
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np
import time
import datetime

from core.supervisor import Supervisor




class Learning(object):
    def __init__(self, environment):
        self.environment = environment
        self.agent = None
        
        
    def produce(self, context, space=None):
        # context is the rotation of the ergo and the ball: "context = environment.get_current_context()"
        if space is None:
            # Autonomous step
            return self.agent.produce(context)
        else:
            # Force space
            assert space in ["s_hand", "s_joystick_1", "s_joystick_2", 's_ergo', "s_ball", "s_light", "s_sound"]
            return self.agent.produce(context, space=space)
            
    def perceive(self, s, m_demo=None, j_demo=False):
        if m_demo is not None:
            # Demonstration of a torso arm trajectory converted to weights with "m_demo = environment.torsodemo2m(m_traj)"
            self.agent.perceive(s, m_demo=m_demo)
        elif j_demo:
            assert len(s) == 102 # [context, s_joystick,...] (no hand trajectory in s)
            self.agent.perceive(s, j_demo=True)
        else:
            # Perception of environment when m was produced
            assert len(s) == 132
            self.agent.perceive(s)
            
    def get_iterations(self): return self.agent.t
    def get_normalized_interests(self): return self.agent.get_normalized_interests()    
    def get_normalized_interests_evolution(self): return self.agent.get_normalized_interests_evolution()
                
    def save(self, log_dir, name, log_normalized_interests=True):        
        data = self.agent.save() 
        filename = os.path.join(log_dir, name + ".pickle")
        with open(filename, 'w') as f:
            pickle.dump(data, f)
        if log_normalized_interests:
            with open(os.path.join(log_dir, name + "_interests" + ".pickle"), 'w') as f:
                pickle.dump(data["normalized_interests_evolution"], f)
    
    def start(self):
        self.agent = Supervisor(self.environment)
         
    def restart(self, log_dir, name, iteration):
        t = time.time()
        self.save(log_dir, name + "_log-restart_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), log_normalized_interests=False)
        print "time save", time.time() - t
        filename = os.path.join(log_dir, name + ".pickle")
        with open(filename, 'r') as f:
            data = pickle.load(f)
        self.start()
        self.agent.forward(data, iteration)
        print "total time restart", time.time() - t

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.get_normalized_interests_evolution(), lw=2)
        ax.legend(["Hand", "Joystick_1", "Joystick_2", "Ergo", "Ball", "Light", "Sound"], ncol=3)
        ax.set_xlabel('Time steps', fontsize=20)
        ax.set_ylabel('Learning progress', fontsize=20)
        plt.show(block=True)
        