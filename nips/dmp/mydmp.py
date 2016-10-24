import numpy as np
from copy import copy

from dmp_discrete import DMPs_discrete
from scipy.optimize import fmin_l_bfgs_b



class MyDMP(object):
    def __init__(self, n_dmps=4, n_bfs=7, timesteps=25, max_params=None):
        
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.timesteps = timesteps
        self.max_params = max_params
        self.bounds = [(-wmax, wmax) for wmax in self.max_params]
         
        self.used = np.array([False]*self.n_dmps + [True]*self.n_bfs*self.n_dmps + [True]*self.n_dmps)
        self.default = np.array([0.] * (self.n_bfs+2) * self.n_dmps)
        self.motor = copy(self.default)
        
        self.dmp = DMPs_discrete(dmps=self.n_dmps, bfs=self.n_bfs, dt=1./self.timesteps)
        

    def trajectory(self, m):
        """
            Compute the motor trajectories from dmp parameters.
        
        """
        self.motor[self.used] = np.array(m)
        self.dmp.y0 = self.motor[:self.dmp.dmps]
        self.dmp.goal = self.motor[-self.dmp.dmps:]
        self.dmp.w = self.motor[self.dmp.dmps:-self.dmp.dmps].reshape(self.dmp.dmps, self.dmp.bfs)
        return self.dmp.rollout(timesteps=self.timesteps)

    def imitate(self, traj, maxfun=1500):
        """
            Imitate a given trajectory with parameter optimization (less than 1 second).
            
        """
        self.dmp.imitate_path(np.transpose(traj))
        x0 = np.array(list(self.dmp.w.flatten()) + list(self.dmp.goal))
        f = lambda w:np.linalg.norm(traj - self.trajectory(w))
        return fmin_l_bfgs_b(f, x0, maxfun=maxfun, bounds=self.bounds, approx_grad=True)[0]
