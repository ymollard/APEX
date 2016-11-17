import numpy as np

from explauto.utils import rand_bounds, bounds_min_max, softmax_choice, prop_choice
from learning_module import LearningModule


class Supervisor(object):
    def __init__(self, environment):
        
        self.environment = environment
        self.conf = self.environment.conf
        
        self.t = 0
        self.modules = {}
        self.chosen_modules = []
        self.progresses_evolution = {}
        self.interests_evolution = {}
        
            
        self.mid_control = ''
        
            
        # Define motor and sensory spaces:
        m_ndims = environment.conf.m_ndims # number of motor parameters
        
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
        
        print
        print "Initialize agent with spaces:"
        print "Motor", self.m_space
        print "Context", self.c_dims
        print "Hand", self.s_hand
        print "Joystick1", self.s_joystick_1
        print "Joystick2", self.s_joystick_2
        print "Ergo", self.s_ergo
        print "Ball", self.s_ball
        print "Light", self.s_light
        print "Sound", self.s_sound
        
        #print "environment.conf", environment.conf
        
        # Create the 6 learning modules:
        self.modules['mod1'] = LearningModule("mod1", self.m_space, self.s_hand, environment.conf)
        self.modules['mod2'] = LearningModule("mod2", self.m_space, self.s_joystick_1, environment.conf)
        self.modules['mod3'] = LearningModule("mod2", self.m_space, self.s_joystick_2, environment.conf)
        self.modules['mod4'] = LearningModule("mod3", self.m_space, [self.c_dims[0]] + self.s_ergo, environment.conf, context_mode=dict(mode='mcs', context_n_dims=1, context_sensory_bounds=[[-1.],[1.]]))
        self.modules['mod5'] = LearningModule("mod4", self.m_space, self.c_dims + self.s_ball, environment.conf, context_mode=dict(mode='mcs', context_n_dims=2, context_sensory_bounds=[[-1., -1.],[1., 1.]]))
        self.modules['mod6'] = LearningModule("mod5", self.m_space, self.c_dims + self.s_light, environment.conf, context_mode=dict(mode='mcs', context_n_dims=2, context_sensory_bounds=[[-1., -1.],[1., 1.]]))
        self.modules['mod7'] = LearningModule("mod6", self.m_space, self.c_dims + self.s_sound, environment.conf, context_mode=dict(mode='mcs', context_n_dims=2, context_sensory_bounds=[[-1., -1.],[1., 1.]]))
    
        self.space2mid = dict(s_hand="mod1", 
                             s_joystick_1="mod2", 
                             s_joystick_2="mod3", 
                             s_ergo="mod4", 
                             s_ball="mod5", 
                             s_light="mod6", 
                             s_sound="mod7")
        
        for mid in self.modules.keys():
            self.progresses_evolution[mid] = []
            self.interests_evolution[mid] = []
        
    
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
                "normalized_interests_evolution":self.get_normalized_interests_evolution()}

        
    def forward(self, data, iteration):
        if iteration > len(data["chosen_modules"]):
            print "\nWARNING: asked to restart from iteration", iteration, "but only", len(data["chosen_modules"]), "are available. Restarting from iteration", len(data["chosen_modules"]), "..."
            iteration = len(data["chosen_modules"])
        self.chosen_modules = data["chosen_modules"][:iteration]
        self.progresses_evolution = data["progresses_evolution"]
        self.interests_evolution = data["interests_evolution"]
        for mid in self.modules.keys():
            self.progresses_evolution[mid] = self.progresses_evolution[mid][:iteration]
            self.interests_evolution[mid] = self.interests_evolution[mid][:iteration]
        self.t = iteration
        if iteration > 0:
            for mid in self.modules.keys():
                if mid == "mod1":
                    self.modules[mid].sensorimotor_model.forward(data["sm_data"][mid], iteration-self.chosen_modules.count("j_demo"))
                else:
                    self.modules[mid].sensorimotor_model.forward(data["sm_data"][mid], iteration)
                    
                self.modules[mid].interest_model.forward(data["im_data"][mid], self.chosen_modules.count(mid), self.progresses_evolution[mid][-1], self.interests_evolution[mid][-1])
    
        
        
    def choose_babbling_module(self, mode='prop'):
        interests = {}
        for mid in self.modules.keys():
            interests[mid] = self.modules[mid].interest()
        
        if mode == 'random':
            mid = np.random.choice(self.interests.keys())
        elif mode == 'greedy':
            eps = 0.2
            if np.random.random() < eps:
                mid = np.random.choice(self.interests.keys())
            else:
                mid = max(interests, key=interests.get)
        elif mode == 'softmax':
            temperature = 0.1
            w = interests.values()
            mid = self.modules.keys()[softmax_choice(w, temperature)]
        
        elif mode == 'prop':
            w = interests.values()
            mid = self.modules.keys()[prop_choice(w, eps=0.2)]
            if (self.t + 1) % 1000 == 0:
                print
                print 'Iteration', self.t +1
                print "Interests", np.array([self.modules[mid].interest() for mid in self.modules.keys()])
                #print "im db n points", [len(self.modules[mid].interest_model.data_xc) for mid in self.modules.keys()]
                #print self.chosen_modules
            #self.interests_evolution.append(w)
        
        self.chosen_modules.append(mid)
        return mid
        
        
    def fast_forward(self, log, forward_im=False):
        #ms_list = []
        for m,s in zip(log.logs['motor'], log.logs['sensori']):
            ms = np.append(m,s)
            self.update_sensorimotor_models(ms)
            #ms_list += [ms]
        for mid, mod in self.modules.iteritems():
            mod.fast_forward_models(log, ms_list=None, from_log_mod=mid, forward_im=forward_im)        
        
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
    def rest_params(self): return self.environment.rest_params()
    def sensory_primitive(self, s): return s    
    def get_m(self, ms): return ms[self.conf.m_dims]
    def get_s(self, ms): return ms[self.conf.s_dims]
                
    def set_ms(self, m, s): return np.array(list(m) + list(s))
            
    def update_sensorimotor_models(self, ms):
        for mid in self.modules.keys():
            self.modules[mid].update_sm(self.modules[mid].get_m(ms), self.modules[mid].get_s(ms))
        
    def produce(self, context, space=None):
        if space is None:
            mid = self.choose_babbling_module()
        else:
            mid = self.space2mid[space]
        self.mid_control = mid
        
        j_sm = self.modules["mod2"].sensorimotor_model
        if self.modules[mid].context_mode is None:
            self.m = self.modules[mid].produce(j_sm=j_sm)
        else:
            self.m = self.modules[mid].produce(context=np.array(context)[range(self.modules[mid].context_mode["context_n_dims"])], j_sm=j_sm)
        return self.m
    
    def inverse(self, mid, s, context):
        s = np.array(list(context[self.modules[mid].context_dims]) + list(s))
        self.mid_control = None
        self.m = self.modules[mid].inverse(s)
        return self.m
    
    def perceive(self, s, m_demo=None, j_demo=False):
        s = self.sensory_primitive(s)
        if m_demo is not None:
            ms = self.set_ms(m_demo, s)
            self.update_sensorimotor_models(ms)
            self.chosen_modules.append("m_demo")
        elif j_demo:
            m0 = [0]*self.conf.m_ndims
            m0s = self.set_ms(m0, s[:2] + [0]*30 + s[2:])
            for mid in self.modules.keys():
                if not (mid == "mod1"): # don't update hand model
                    self.modules[mid].update_sm(self.modules[mid].get_m(m0s), self.modules[mid].get_s(m0s))
            self.chosen_modules.append("j_demo")
        else:
            ms = self.set_ms(self.m, s)
            self.update_sensorimotor_models(ms)
            if self.mid_control is not None:
                self.modules[self.mid_control].update_im(self.modules[self.mid_control].get_m(ms), self.modules[self.mid_control].get_s(ms))
        self.t = self.t + 1
        
        for mid in self.modules.keys():
            self.progresses_evolution[mid].append(self.modules[mid].progress())
            self.interests_evolution[mid].append(self.modules[mid].interest())

    def get_normalized_interests_evolution(self):
        data = np.transpose(np.array([self.interests_evolution[mid] for mid in ["mod1", "mod2", "mod3", "mod4", "mod5", "mod6", "mod7"]]))
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
        
