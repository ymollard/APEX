
import os
import sys
import pickle
import matplotlib.pyplot as plt
import numpy as np

sys.path.append('../../')
from nips.environment.environment import TestEnvironment
from nips.learning.supervisor import Supervisor




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
            assert space in ["s_hand", "s_joystick", 's_ergo', "s_ball", "s_light", "s_sound"]
            return self.agent.produce(context, space=space)
            
    def perceive(self, s, m_demo=None, j_demo=False):
        if m_demo is not None:
            # Demonstration of a torso arm trajectory converted to weights with "m_demo = environment.torsodemo2m(m_traj)"
            self.agent.perceive(s, m_demo=m_demo)
        elif j_demo:
            assert len(s) == 82 # [context, s_joystick,...] (no hand trajectory in s)
            self.agent.perceive(s, j_demo=True)
        else:
            # Perception of environment when m was produced
            assert len(s) == 112
            self.agent.perceive(s)
            
    def get_iterations(self):
        return self.learning.agent.t
    
    def get_current_interests(self):
        interests = {}
        for mid in self.learning.agent.modules.keys():
            interests[mid] = self.learning.modules[mid].interest()
        return interests
                
    def save(self, log_dir, name):        
        data = self.agent.save() 
        filename = os.path.join(log_dir, name)
        with open(filename, 'w') as f:
            pickle.dump(data, f)
    
    def start(self):
        self.agent = Supervisor(self.environment)
         
    def restart(self, log_dir, name, iteration):
        filename = os.path.join(log_dir, name)
        with open(filename, 'r') as f:
            data = pickle.load(f)
        self.start()
        self.agent.forward(data, iteration)

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(np.transpose(np.array(self.agent.interests_evolution.values())), lw=2)
        ax.legend(["s_hand", "s_joystick", "s_ergo", "s_ball", "s_light", "s_sound"], ncol=3)
        ax.set_xlabel('Time steps', fontsize=20)
        ax.set_ylabel('Learning progress', fontsize=20)
        plt.show(block=True)
        
        
        
if __name__ == "__main__":
    
    print "Create environment"
    environment = TestEnvironment()
    
    print "Create agent"
    learning = Learning(environment)
    learning.start()
    
    print
    print "Do 2000 autonomous steps:" 
    for i in range(2000):
        context = environment.get_current_context()
        m = learning.produce(context)
        s = environment.update(m)
        learning.perceive(s)
        
#     print "Do 1 arm demonstration"
#     m_demo_traj = np.zeros((25, 4)) + 0.001
#     m_demo = environment.torsodemo2m(m_demo_traj)
#     s = environment.update(m_demo)
#     learning.perceive(s, m_demo=m_demo)
    
    print
    print "Do 1 joystick demonstration to show how to produce light"
    j_demo = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.2, 0.001, 0., 0., 0., 0., 0.]
    s = environment.get_current_context() + j_demo + [0.2]*20 + [0.2]*20 + [0.1]*10 + [0.]*10
    #print "j_demo", s
    learning.perceive(s, j_demo=True)
    
    print "Now ask to produce light..."
    learning.produce(environment.get_current_context(), "s_light")
    
    
    
    print
    print "Saving current data to file"
    learning.save("../../data", "test.pickle")
    
#     print "Data before saving"
#     print learning.agent.t
#     print learning.agent.interests_evolution["mod1"][-10:]
#     print learning.agent.progresses_evolution["mod1"][-10:]
#     print learning.agent.chosen_modules[-10:]
#     print len(learning.agent.modules["mod1"].sensorimotor_model.model.imodel.fmodel.dataset)
#     print len(learning.agent.modules["mod2"].sensorimotor_model.model.imodel.fmodel.dataset)
#     print learning.agent.modules["mod1"].interest_model.current_interest
    
    print
    print "Do 2000 autonomous steps:" 
    for i in range(2000):
        context = environment.get_current_context()
        m = learning.produce(context)
        s = environment.update(m)
        learning.perceive(s)
    
    print "Rebuilding agent from file"
    learning.restart("../../data", "test.pickle", 2001)
        
#     print "Data after rebuilding"
#     print learning.agent.t
#     print learning.agent.interests_evolution["mod1"][-10:]
#     print learning.agent.progresses_evolution["mod1"][-10:]
#     print learning.agent.chosen_modules[-10:]
#     print len(learning.agent.modules["mod1"].sensorimotor_model.model.imodel.fmodel.dataset)
#     print len(learning.agent.modules["mod2"].sensorimotor_model.model.imodel.fmodel.dataset)
#     print learning.agent.modules["mod1"].interest_model.current_interest
    
    print
    print "Do 2000 autonomous steps:" 
    for i in range(2000):
        context = environment.get_current_context()
        m = learning.produce(context)
        s = environment.update(m)
        learning.perceive(s)
        
    print "\nPloting interests..."
    learning.plot()