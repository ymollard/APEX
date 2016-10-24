import numpy as np

from explauto.utils import bounds_min_max
from explauto.environment.environment import Environment
from nips.dmp.mydmp import MyDMP



class TestEnvironment(Environment):
    def __init__(self, m_mins, m_maxs, s_mins, s_maxs):
        
        Environment.__init__(self, m_mins, m_maxs, s_mins, s_maxs)
        
        
        # DMP PARAMETERS
        self.n_dmps=4
        self.n_bfs=7
        self.timesteps=25
        self.max_params = np.array([300.] * self.n_bfs * self.n_dmps + [1.] * self.n_dmps)
        self.motor_dmp = MyDMP(n_dmps=self.n_dmps, n_bfs=self.n_bfs, timesteps=self.timesteps, max_params=self.max_params)
        
        # SPACES
        self.hand = 30*[0.]
        self.joystick = 20*[0.]
        self.ergo = 20*[0.]
        self.ball = 20*[0.]
        self.light = 10*[0.]
        self.sound = 10*[0.]
        
        # CONTEXT
        self.ergo_theta = 0.
        self.ball_theta = 0.
        self.current_context = [self.ergo_theta, self.ball_theta]
        
        
    def compute_motor_command(self, m):
        return bounds_min_max(m, self.conf.m_mins, self.conf.m_maxs)


    def compute_dmp(self, m):
        return bounds_min_max(self.motor_dmp.trajectory(np.array(m) * self.max_params), self.n_dmps * [-1.], self.n_dmps * [1.]) # DMP TO TUNE 


    def compute_sensori_effect(self, m):
        
        # SAMPLE DMP
        m_dyn = self.compute_dmp(m)
        #print "m_dyn", m_dyn
        
        # COMPUTE PERCEPTION
        self.hand = list(m_dyn[:25, 0]) + list(m_dyn[:5, 1])
        self.joystick = list(np.array(self.hand[:20]) - np.array(self.hand[10:]))
        self.ergo = list(self.ergo_theta - np.array(self.joystick))
        self.ergo_theta = ((self.ergo[-1] + np.pi) % (2. * np.pi)) - np.pi  
        self.ball = list(self.ball_theta - np.array(self.ergo))
        self.ball_theta = ((self.ball[-1] + np.pi) % (2. * np.pi)) - np.pi  
        self.light = list(np.array(self.ball[:10]) - np.array(self.ball[10:]))
        self.sound = list(np.array(self.ball[2:12]) - np.array(self.ball[10:]))
        self.current_context = [self.ergo_theta, self.ball_theta]
        
#         print "hand", self.hand
#         print "joystick", self.joystick
#         print "ergo", self.ergo
#         print "ball", self.ball
#         print "light", self.light
#         print "sound", self.sound
#         print "current_context", self.current_context
        
        return list(self.hand + self.joystick + self.ergo + self.ball + self.light + self.sound)
    

