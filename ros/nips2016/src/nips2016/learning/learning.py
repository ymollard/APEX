
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np
import time
import datetime

from core.supervisor import Supervisor




class Learning(object):
    def __init__(self, config, n_motor_babbling=0, explo_noise=0.1, choice_eps=0.2):
        self.config = config
        self.n_motor_babbling = n_motor_babbling
        self.explo_noise = explo_noise
        self.choice_eps = choice_eps
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
            assert len(m_demo) == 32, len(m_demo)
            assert len(s) == 132, len(s)
            # Demonstration of a torso arm trajectory converted to weights with "m_demo = environment.torsodemo2m(m_traj)"
            self.agent.perceive(s, m_demo=m_demo)
        elif j_demo:
            assert len(s) == 102, len(s) # [context, s_joystick,...] (no hand trajectory in s)
            self.agent.perceive(s, j_demo=True)
        else:
            # Perception of environment when m was produced
            assert len(s) == 132, len(s)
            self.agent.perceive(s)
            
    def get_iterations(self): return self.agent.t
    def get_normalized_interests(self): return self.agent.get_normalized_interests()    
    def get_normalized_interests_evolution(self): return self.agent.get_normalized_interests_evolution()
    def get_last_focus(self): return self.agent.get_last_focus()
    def get_space_names(self): return self.agent.get_space_names()
    def motor_babbling(self): return self.agent.motor_babbling()
    
    def move_hand(self, context, direction="up"):
        if direction=="up":
            return self.agent.inverse("mod1", [0., 0., 0.,
                                               0., 0., 0., 
                                               0., 0., 0.5,
                                               0., 0., 0.5,
                                               0., 0., 1.,
                                               0., 0., 1., 
                                               0., 0., 1.,
                                               0., 0., 1.,
                                               0., 0., 1., 
                                               0., 0., 1.], context)
        elif direction=="forward":
            return self.agent.inverse("mod1", [0., 0., 0., 
                                               0., 0., 0., 
                                               0.5, 0., 0., 
                                               0.5, 0., 0., 
                                               1., 0., 0., 
                                               1., 0., 0., 
                                               1., 0., 0., 
                                               1., 0., 0., 
                                               1., 0., 0., 
                                               1., 0., 0.,], context)
        elif direction=="right":
            return self.agent.inverse("mod1", [0., 0., 0., 
                                               0., 0., 0., 
                                               0., -0.5, 0., 
                                               0., -0.5, 0., 
                                               0., -1., 0., 
                                               0., -1., 0., 
                                               0., -1., 0., 
                                               0., -1., 0., 
                                               0., -1., 0., 
                                               0., -1., 0.,], context)
        elif direction=="left":
            return self.agent.inverse("mod1", [0., 0., 0., 
                                               0., 0., 0., 
                                               0., 0.5, 0., 
                                               0., 0.5, 0., 
                                               0., 1., 0., 
                                               0., 1., 0., 
                                               0., 1., 0., 
                                               0., 1., 0., 
                                               0., 1., 0., 
                                               0., 1., 0.,], context)
        else:
            raise NotImplementedError
            
        
    def motor_move_joystick_1(self, context, direction="forward"):
        if direction=="forward":
            return self.agent.inverse("mod2", [-1., 0., 
                                               -1., 0., 
                                               0., 0., 
                                               1., 0., 
                                               1., 0., 
                                               1., 0., 
                                               1., 0., 
                                               0., 0., 
                                               -1., 0., 
                                               -1., 0.], context)
        elif direction=="right":
            return self.agent.inverse("mod2", [-1., 0., 
                                               -1., 0., 
                                               -1., 0., 
                                               -1., 1., 
                                               -1., 1., 
                                               -1., 1., 
                                               -1., 1., 
                                               -1., 0., 
                                               -1., 0., 
                                               -1., 0.], context)
        elif direction=="left":
            return self.agent.inverse("mod2", [-1., 0., 
                                               -1., 0., 
                                               -1., 0., 
                                               -1., -1., 
                                               -1., -1., 
                                               -1., -1., 
                                               -1., -1., 
                                               -1., 0., 
                                               -1., 0., 
                                               -1., 0.], context)  
        else:
            raise NotImplementedError
              
    def motor_move_joystick_2(self, context, direction="forward"):
        if direction=="forward":
            return self.agent.inverse("mod3", [0., -1., 
                                               0., -1., 
                                               0., 0., 
                                               0., 1., 
                                               0., 1., 
                                               0., 1., 
                                               0., 1., 
                                               0., 0., 
                                               0., -1., 
                                               0., -1.], context)
        elif direction=="right":
            return self.agent.inverse("mod3", [0., -1., 
                                               0., -1., 
                                               0., -1., 
                                               1., -1., 
                                               1., -1., 
                                               1., -1., 
                                               1., -1., 
                                               0., -1., 
                                               0., -1., 
                                               0., -1.], context)
        elif direction=="left":
            return self.agent.inverse("mod3", [0., -1., 
                                               0., -1., 
                                               0., -1., 
                                               -1., -1., 
                                               -1., -1., 
                                               -1., -1., 
                                               -1., -1., 
                                               0., -1., 
                                               0., -1., 
                                               0., -1.], context)
        else:
            raise NotImplementedError
    
    def motor_move_ergo(self, context, direction="right"):
        angle = context[0]
        if direction=="right":
            return self.agent.inverse("mod4", [angle, -1.,
                                               angle, -1.,
                                               ((angle+1.+0.25) % 2.)- 1., 0.,
                                               ((angle+1.+0.50) % 2.)- 1., 1.,
                                               ((angle+1.+0.75) % 2.)- 1., 1.,
                                               ((angle+1.+1.00) % 2.)- 1., 1.,
                                               ((angle+1.+1.25) % 2.)- 1., 1.,
                                               ((angle+1.+1.75) % 2.)- 1., 0.,
                                               ((angle+1.+2.) % 2.)- 1., -1.,
                                               ((angle+1.+2.) % 2.)- 1., -1.], context)
        elif direction=="left":
            return self.agent.inverse("mod4", [angle, -1.,
                                               angle, -1.,
                                               ((angle+1.-0.25) % 2.)- 1., 0.,
                                               ((angle+1.-0.50) % 2.)- 1., 1.,
                                               ((angle+1.-0.75) % 2.)- 1., 1.,
                                               ((angle+1.-1.00) % 2.)- 1., 1.,
                                               ((angle+1.-1.25) % 2.)- 1., 1.,
                                               ((angle+1.-1.75) % 2.)- 1., 0.,
                                               ((angle+1.-2.) % 2.)- 1., -1.,
                                               ((angle+1.-2.) % 2.)- 1., -1.], context)
        else:
            raise NotImplementedError
        
    def motor_move_ball(self, context, direction="right"):
        angle = context[1]
        if direction=="right":
            return self.agent.inverse("mod5", [angle, -1.,
                                               angle, -1.,
                                               ((angle+1.+0.25) % 2.)- 1., 0.,
                                               ((angle+1.+0.50) % 2.)- 1., 1.,
                                               ((angle+1.+0.75) % 2.)- 1., 1.,
                                               ((angle+1.+1.00) % 2.)- 1., 1.,
                                               ((angle+1.+1.25) % 2.)- 1., 1.,
                                               ((angle+1.+1.75) % 2.)- 1., 0.,
                                               ((angle+1.+2.) % 2.)- 1., -1.,
                                               ((angle+1.+2.) % 2.)- 1., -1.], context)
        elif direction=="left":
            return self.agent.inverse("mod5", [angle, -1.,
                                               angle, -1.,
                                               ((angle+1.-0.25) % 2.)- 1., 0.,
                                               ((angle+1.-0.50) % 2.)- 1., 1.,
                                               ((angle+1.-0.75) % 2.)- 1., 1.,
                                               ((angle+1.-1.00) % 2.)- 1., 1.,
                                               ((angle+1.-1.25) % 2.)- 1., 1.,
                                               ((angle+1.-1.75) % 2.)- 1., 0.,
                                               ((angle+1.-2.) % 2.)- 1., -1.,
                                               ((angle+1.-2.) % 2.)- 1., -1.], context)
        else:
            raise NotImplementedError
            
    
    def motor_make_light(self, context):
        return self.agent.inverse("mod6", [-1., -1., 0., 1., 1., 1., 1., 0., -1., -1.], context)
    
    def motor_make_sound(self, context):
        return self.agent.inverse("mod7", [-1., -1., 0., 1., 1., 1., 1., 0., -1., -1.], context)
        
    
    def get_data_from_file(self, log_dir, name):
        filename = os.path.join(log_dir, name + ".pickle")
        with open(filename, 'r') as f:
            data = pickle.load(f)
        return data
                
    def save(self, log_dir, name, log_normalized_interests=True):        
        data = self.agent.save() 
        filename = os.path.join(log_dir, name + ".pickle")
        with open(filename, 'w') as f:
            pickle.dump(data, f)
        if log_normalized_interests:
            with open(os.path.join(log_dir, name + "_interests" + ".pickle"), 'w') as f:
                pickle.dump(data["normalized_interests_evolution"], f)
    
    def start(self):
        self.agent = Supervisor(self.config, 
                                n_motor_babbling=self.n_motor_babbling, 
                                explo_noise=self.explo_noise, 
                                choice_eps=self.choice_eps)
        
    def restart_from_end_of_file(self, log_dir, name):
        data = self.get_data_from_file(log_dir, name)
        self.start()
        self.agent.forward(data, len(data["chosen_modules"]))
    
    def restart_from_file(self, log_dir, name, iteration):
        #self.save(log_dir, name + "_log-restart_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), log_normalized_interests=False)
        data = self.get_data_from_file(log_dir, name)
        self.start()
        self.agent.forward(data, iteration)

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.get_normalized_interests_evolution(), lw=2)
        ax.legend(["Hand", "Joystick_1", "Joystick_2", "Ergo", "Ball", "Light", "Sound"], ncol=3)
        ax.set_xlabel('Time steps', fontsize=20)
        ax.set_ylabel('Learning progress', fontsize=20)
        plt.show(block=True)
        