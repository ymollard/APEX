'''
Copyright (C) 2013 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

from dmp import DMPs

import numpy as np

class DMPs_discrete(DMPs):
    """An implementation of discrete DMPs"""

    def __init__(self, **kwargs): 
        """
        """

        # call super class constructor
        super(DMPs_discrete, self).__init__(pattern='discrete', **kwargs)

        self.gen_centers()

        # set variance of Gaussian basis functions
        # trial and error to find this spacing
        self.h = np.ones(self.bfs) * 8. * self.bfs**2 / self.c**1.5
        #self.h = np.ones(self.bfs) * self.bfs**1.5 / self.c

        self.check_offset()
        self.psi_track = None
        
    def gen_centers(self):
        """Set the centre of the Gaussian basis 
        functions be spaced evenly throughout run time"""

        '''x_track = self.cs.discrete_rollout()
        t = np.arange(len(x_track))*self.dt
        # choose the points in time we'd like centers to be at
        c_des = np.linspace(0, self.cs.run_time, self.bfs)
        self.c = np.zeros(len(c_des))
        for ii, point in enumerate(c_des): 
            diff = abs(t - point)
            self.c[ii] = x_track[np.where(diff == min(diff))[0][0]]'''

        # MODIFIED
        first = 0. #relative to runtime [0,1]
        last = 0.8
        des_c = np.linspace(last,first,self.bfs) 

        self.c = np.ones(len(des_c)) 
        for n in range(len(des_c)): 
            # x = exp(-c), solving for c
            self.c[n] = np.exp(-self.cs.run_time*des_c[n])

    def gen_front_term(self, x, dmp_num):
        """Generates the diminishing front term on 
        the forcing term.

        x float: the current value of the canonical system
        dmp_num int: the index of the current dmp
        """
        return x# * (self.goal[dmp_num] - self.y0[dmp_num])

    def gen_goal(self, y_des): 
        """Generate the goal for path imitation. 
        For rhythmic DMPs the goal is the average of the 
        desired trajectory.
    
        y_des np.array: the desired trajectory to follow
        """

        return y_des[:,-1].copy()
    
    def gen_psi(self, x):
        """Generates the activity of the basis functions for a given 
        state of the canonical system.

        x float: the current state of the canonical system
        """
        
        return np.exp(-self.h * (x - self.c)**2)
        
    def gen_psi(self, x):
        """Generates the activity of the basis functions for a given 
        canonical system rollout. 
        
        x float, array: the canonical system state or path
        """
   
        if isinstance(x, np.ndarray):
            x = x[:,None]
        return np.exp(-self.h * (x - self.c)**2)

    def gen_weights(self, f_target):
        """Generate a set of weights over the basis functions such 
        that the target forcing term trajectory is matched.
        
        f_target np.array: the desired forcing term trajectory
        """

        # calculate x and psi   
        #x_track = self.cs.rollout()
        x_track = self.cs.x_track
        psi_track = self.gen_psi(x_track)
        self.psi_track = psi_track

        #efficiently calculate weights for BFs using weighted linear regression
        self.w = np.zeros((self.dmps, self.bfs))
        for d in range(self.dmps):
            # spatial scaling term
            k = 1.#(self.goal[d] - self.y0[d])
            for b in range(self.bfs):
                numer = np.sum(x_track * psi_track[:,b] * f_target[:,d])
                denom = np.sum(x_track**2 * psi_track[:,b])
                self.w[d,b] = numer / (k * denom)
