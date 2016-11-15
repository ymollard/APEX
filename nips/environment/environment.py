import numpy as np

from explauto.utils import bounds_min_max
from explauto.environment.environment import Environment
from explauto.environment.context_environment import ContextEnvironment
from nips.dmp.mydmp import MyDMP



class TestNCEnvironment(Environment):
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
        self.joystick1 = 20*[0.]
        self.joystick2 = 20*[0.]
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
        return bounds_min_max(self.motor_dmp.trajectory(np.array(m) * self.max_params), self.n_dmps * [-1.], self.n_dmps * [1.])
        
    def torsodemo2m(self, m_traj):
        # m_traj is the trajectory of the 4 motors: must be shaped 25*4
        return self.motor_dmp.imitate(m_traj) / self.max_params

    def compute_sensori_effect(self, m):
        
        # SAMPLE DMP
        m_dyn = self.compute_dmp(m)
        #print "m_dyn", m_dyn
        
        # COMPUTE PERCEPTION
        self.hand = list(np.cos(2*np.pi*m_dyn[:10, 0])+np.sin(2*np.pi*m_dyn[:10, 1])) + list(np.cos(2*np.pi*m_dyn[:10, 2])+np.sin(2*np.pi*m_dyn[10:20, 3])) + list(np.cos(2*np.pi*m_dyn[10:20, 0])+np.sin(2*np.pi*m_dyn[:10, 3]))
        
        h = self.hand
        j1 = np.array(h)[:10]
        j1[j1 < -0.25] = 0
        j1[j1 > 0.25] = 0
        j1 = j1 * 4
        j2 = np.array(h)[10:20]
        j2[j2 < -0.25] = 0
        j2[j2 > 0.25] = 0
        j2 = j2 * 4
        self.joystick1 = list(j1) + list(j2)
        
        self.ergo = list([0.]*10) + list((self.ergo_theta+np.cumsum(self.joystick1[:10])+ np.pi) % (2. * np.pi) - np.pi)
        
        self.ergo_theta = self.ergo[-1]
        
        d = np.abs(np.array(self.ergo[10:]) - self.ball_theta)
        d[d > 0.25] = 1
        d = 1 - d 
        self.ball = list([0.]*10) + list((self.ball_theta+np.cumsum(d)+ np.pi) % (2. * np.pi) - np.pi)
        self.ball_theta = self.ball[-1]

        if 0.2 < np.linalg.norm(self.joystick1) < 0.201:
            self.light = list(10.*m_dyn[:10, 0])
            print "LIGHT"
        else:
            self.light = list([0.]*10)
        self.sound = list([0.]*10)
        self.current_context = [self.ergo_theta, self.ball_theta]
        
#         print
#         print "hand", self.hand
#         print "joystick", self.joystick
#         print "ergo", self.ergo
#         print "ball", self.ball
#         print "light", self.light
#         print "sound", self.sound
#         print "current_context", self.current_context
        
        return list(self.hand + self.joystick1 + self.joystick2 + self.ergo + self.ball + self.light + self.sound)
    


class TestEnvironment(ContextEnvironment):
    def __init__(self):
        env_cls = TestNCEnvironment
        env_conf = dict(m_mins=[-1.]*32, 
                        m_maxs=[1.]*32, 
                        s_mins=[-1.]*130, 
                        s_maxs=[1.]*130)
        context_mode = dict(mode='mcs',
                            context_n_dims=2,
                            context_sensory_bounds=[[-1., -1.],[1., 1.]])
        
        ContextEnvironment.__init__(self, env_cls, env_conf, context_mode)
        
    def torsodemo2m(self, m_traj): return self.env.torsodemo2m(m_traj)