import numpy as np
from copy import copy

from dmp_discrete import DMPs_discrete
from scipy.optimize import fmin_bfgs



class DmpPrimitive(object):
    def __init__(self):
        
        self.n_dmps = 4
        self.n_bfs = 7
        self.timesteps = 25
        self.used = np.array([False]*self.n_dmps + [True]*self.n_bfs*self.n_dmps + [True]*self.n_dmps)
        self.default = np.array([0.] * (self.n_bfs+2) * self.n_dmps)
        self.motor = copy(self.default)
        
        self.dmp = DMPs_discrete(dmps=self.n_dmps, bfs=self.n_bfs, dt=1./self.timesteps)
        

    def trajectory(self, m):
        self.motor[self.used] = m
        self.dmp.y0 = self.motor[:self.dmp.dmps]
        self.dmp.goal = self.motor[-self.dmp.dmps:]
        self.dmp.w = self.motor[self.dmp.dmps:-self.dmp.dmps].reshape(self.dmp.dmps, self.dmp.bfs)
        return self.dmp.rollout(timesteps=self.timesteps)

    def imitate(self, traj, maxiter=50):
        self.dmp.imitate_path(np.transpose(traj))
        x0 = np.array(list(self.dmp.w.flatten()) + list(self.dmp.goal))
        
        return fmin_bfgs(lambda w:np.linalg.norm(traj - self.trajectory(w)), x0, maxiter=maxiter)
