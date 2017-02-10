import numpy as np
import time
from explauto.utils import rand_bounds, bounds_min_max, softmax_choice, prop_choice
from explauto.utils.config import make_configuration
from learning_module import LearningModule


class FGB(object):
    def __init__(self, config, n_motor_babbling=0, explo_noise=0.1):
        
        self.config = config
        self.n_motor_babbling = n_motor_babbling
        self.explo_noise = explo_noise
        
        self.conf = make_configuration(**config)
        
        self.t = 0
        self.mid_control = ""
            
        # Define motor and sensory spaces:
        m_ndims = self.conf.m_ndims # number of motor parameters
        
        self.m_space = range(m_ndims)
        self.c_dims = range(m_ndims, m_ndims+2)
        self.s_hand = range(m_ndims+2, m_ndims+32)
        self.s_joystick_1 = range(m_ndims+32, m_ndims+52)
        self.s_joystick_2 = range(m_ndims+52, m_ndims+72)
        self.s_ergo = range(m_ndims+72, m_ndims+92)
        self.s_ball = range(m_ndims+92, m_ndims+112)
        self.s_light = range(m_ndims+112, m_ndims+122)
        self.s_sound = range(m_ndims+122, m_ndims+132)
        
        self.s_spaces = dict(s_hand=self.s_hand, 
                             s_joystick_1=self.s_joystick_1, 
                             s_joystick_2=self.s_joystick_2, 
                             s_ergo=self.s_ergo, 
                             s_ball=self.s_ball, 
                             s_light=self.s_light, 
                             s_sound=self.s_sound)
        
        
        self.modules = {}
        self.modules["mod"] = LearningModule("mod", self.m_space, self.c_dims + self.s_hand+self.s_joystick_1+self.s_joystick_2+self.s_ergo+self.s_ball+self.s_light+self.s_sound, self.conf, context_mode=dict(mode='mcs', context_n_dims=2, context_sensory_bounds=[[-1., -1.],[1., 1.]]), explo_noise=self.explo_noise, normalize_interests=self.normalize_interests)

        
        for mid in self.modules.keys():
            self.progresses_evolution[mid] = []
            self.interests_evolution[mid] = []
    
    def get_last_focus(self): return self.mid_to_space(self.mid_control) if self.mid_control else ""
    
    def save(self):
        sm_data = {}
        im_data = {}
        for mid in self.modules.keys():
            sm_data[mid] = self.modules[mid].sensorimotor_model.save()
            im_data[mid] = self.modules[mid].interest_model.save()            
        return {"sm_data":sm_data,
                "im_data":im_data,
                "chosen_modules":self.chosen_modules,
                "progresses_evolution":self.progresses_evolution,
                "interests_evolution":self.interests_evolution,
                "normalized_interests_evolution":self.get_normalized_interests_evolution(),
                "normalize_interests":self.normalize_interests}

        
    def forward(self, data, iteration):
        if iteration > len(data["chosen_modules"]):
            print "\nWARNING: asked to restart from iteration", iteration, "but only", len(data["chosen_modules"]), "are available. Restarting from iteration", len(data["chosen_modules"]), "..."
            iteration = len(data["chosen_modules"])
        if iteration < 0:
            iteration = len(data["chosen_modules"])
        self.chosen_modules = data["chosen_modules"][:iteration]
        self.progresses_evolution = data["progresses_evolution"]
        self.interests_evolution = data["interests_evolution"]
        for mid in self.modules.keys():
            self.progresses_evolution[mid] = self.progresses_evolution[mid][:iteration]
            self.interests_evolution[mid] = self.interests_evolution[mid][:iteration]
        self.t = iteration
        if data.has_key("normalize_interests"):
            self.normalize_interests = data["normalize_interests"]
        if iteration > 0:
            for mid in self.modules.keys():
                if mid == "mod1":
                    self.modules[mid].sensorimotor_model.forward(data["sm_data"][mid], iteration-self.chosen_modules.count("j_demo"))
                else:
                    self.modules[mid].sensorimotor_model.forward(data["sm_data"][mid], iteration)
                    
                self.modules[mid].interest_model.forward(data["im_data"][mid], self.chosen_modules.count(mid), self.progresses_evolution[mid][-1], self.interests_evolution[mid][-1])
    
        
        
    def choose_babbling_module(self):
        self.chosen_modules.append("mod")
        return "mod"   
        
    def eval_mode(self): 
        self.sm_modes = {}
        for mod in self.modules.values():
            self.sm_modes[mod.mid] = mod.sensorimotor_model.mode
            mod.sensorimotor_model.mode = 'exploit'
                
    def learning_mode(self): 
        for mod in self.modules.values():
            mod.sensorimotor_model.mode = self.sm_modes[mod.mid]
                
    def check_bounds_dmp(self, m_ag):return bounds_min_max(m_ag, self.conf.m_mins, self.conf.m_maxs)
    def motor_primitive(self, m): return m
    def sensory_primitive(self, s): return s
    def get_m(self, ms): return ms[self.conf.m_dims]
    def get_s(self, ms): return ms[self.conf.s_dims]
    
    def motor_babbling(self):
        self.m = self.modules["mod"].motor_babbling()
        return self.m
    
    def set_ms(self, m, s): return np.array(list(m) + list(s))
            
    def update_sensorimotor_models(self, ms):
        for mid in self.modules.keys():
            self.modules[mid].update_sm(self.modules[mid].get_m(ms), self.modules[mid].get_s(ms))
        
    def increase_interest(self, mid):
        pass
            
    def produce(self, context, space=None):
        if self.t < self.n_motor_babbling:
            self.mid_control = None
            self.chosen_modules.append("motor_babbling")
            return self.motor_babbling()
        else:
            mid = self.choose_babbling_module(self.babbling_mode)
            self.mid_control = mid
            self.m = self.modules[mid].produce(context=np.array(context)[range(self.modules[mid].context_mode["context_n_dims"])])
            return self.m
    
    def inverse(self, mid, s, context):
        if self.modules[mid].context_mode is not None:
            s = np.array(list(context[:self.modules[mid].context_mode["context_n_dims"]]) + list(s))
        else:
            s = np.array(s)
        self.mid_control = None
        self.chosen_modules.append("inverse_" + mid)
        self.m = self.modules[mid].inverse(s)
        return self.m
    
    def dist_angle(self, a1, a2): return min(abs(a1 - a2), 2 - abs(a1 - a2))
    
    def ball_moves(self, s):
        a1 = s[16]
        a2 = s[18]
        # print "ball end angular speed", self.dist_angle(a1, a2)
        return self.dist_angle(a1, a2) > 0.1
    
    def perceive(self, s, m_demo=None, j_demo=False):
        s = self.sensory_primitive(s)
        #print "perceive len(s)", len(s), s[92:112]
        if j_demo or self.ball_moves(s[92:112]):
            time.sleep(3)
        if m_demo is not None:
            return False
        elif j_demo:
            return False
        else:
            if not hasattr(self, "m"):
                return False
            ms = self.set_ms(self.m, s)
            self.update_sensorimotor_models(ms)
            if self.mid_control is not None:
                self.modules[self.mid_control].update_im(self.modules[self.mid_control].get_m(ms), self.modules[self.mid_control].get_s(ms))
        self.t = self.t + 1
        
        for mid in self.modules.keys():
            self.progresses_evolution[mid].append(self.modules[mid].progress())
            self.interests_evolution[mid].append(self.modules[mid].interest())
                
        return True
    
    
    def produce_goal(self, context, goal):
        return self.motor_babbling()
    

    def get_normalized_interests_evolution(self):
        data = np.transpose(np.array([self.interests_evolution[mid] for mid in ["mod"]]))
        data_sum = data.sum(axis=1)
        data_sum[data_sum==0.] = 1.
        return data / data_sum.reshape(data.shape[0],1)
    
    def get_normalized_interests(self):
        interests = {}
        for mid in self.modules.keys():
            interests[mid] = self.modules[mid].interest()
            
        s = sum(interests.values())
        if s > 0:
            for mid in self.modules.keys():
                interests[mid] = interests[mid] / s
        return interests
        
