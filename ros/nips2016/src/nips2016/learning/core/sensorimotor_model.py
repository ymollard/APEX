
import numpy as np
from numpy import array
from explauto.exceptions import ExplautoBootstrapError
from explauto.sensorimotor_model.non_parametric import NonParametric
from explauto.utils import bounds_min_max
from explauto.utils import rand_bounds


class DemonstrableNN(NonParametric):
    def __init__(self, conf, sigma_explo_ratio=0.1, fwd='LWLR', inv='L-BFGS-B', **learner_kwargs):
        self.demonstrated = []
        NonParametric.__init__(self, conf, sigma_explo_ratio, fwd, inv, **learner_kwargs)
        
    def save(self):
        return [[self.model.imodel.fmodel.dataset.get_x(i) for i in range(len(self.model.imodel.fmodel.dataset))],
                [self.model.imodel.fmodel.dataset.get_y(i) for i in range(len(self.model.imodel.fmodel.dataset))]]
    
    def forward(self, data, iteration):
        self.model.imodel.fmodel.dataset.add_xy_batch(data[0], data[1])
        self.t = iteration

    def infer(self, in_dims, out_dims, x):
        if self.t < max(self.model.imodel.fmodel.k, self.model.imodel.k):
            raise ExplautoBootstrapError

        if in_dims == self.m_dims and out_dims == self.s_dims:  # forward
            return array(self.model.predict_effect(tuple(x))), -1

        elif in_dims == self.s_dims and out_dims == self.m_dims:  # inverse
            if not self.bootstrapped_s:
                # If only one distinct point has been observed in the sensory space, then we output a random motor command
                return rand_bounds(np.array([self.m_mins,
                                             self.m_maxs]))[0], -1
            else:
                self.mean_explore = array(self.model.infer_order(tuple(x)))
                idx = -1
                # Check if nearest s was demonstrated
                if np.linalg.norm(self.mean_explore) == 0:
                    idx = self.model.imodel.fmodel.dataset.nn_y(x)[1][0]
                    #print "demonstrated idx", idx
                if self.mode == 'explore':
                    r = self.mean_explore
                    r[self.sigma_expl > 0] = np.random.normal(r[self.sigma_expl > 0], self.sigma_expl[self.sigma_expl > 0])
                    res = bounds_min_max(r, self.m_mins, self.m_maxs)
                    return res, idx
                else:  # exploit'
                    return array(self.model.infer_order(tuple(x))), idx

        else:
            raise NotImplementedError
        
    def inverse_idx(self, idx):
        #print "Retrieve joystick demonstration"
        s = self.model.imodel.fmodel.dataset.get_y(idx)
        #print "s demo", s
        _, idxs = self.model.imodel.fmodel.dataset.nn_y(s, k=1000)
        # Find nearest s that was not a demo
        for idx in idxs:
            m = self.model.imodel.fmodel.dataset.get_x(idx)
            if np.linalg.norm(m) > 0:
                break
#         print "nn idx", idx
#         print "snn", self.model.imodel.fmodel.dataset.get_y(idx)
#         print "m", m 
        r = m
        r[self.sigma_expl > 0] = np.random.normal(r[self.sigma_expl > 0], self.sigma_expl[self.sigma_expl > 0])
        res = bounds_min_max(r, self.m_mins, self.m_maxs)
        return res
    
    def update(self, m, s):
        self.model.add_xy(tuple(m), tuple(s))
        self.t += 1
        if not self.bootstrapped_s and self.t > 1:
            if not (list(s[2:]) == list(self.model.imodel.fmodel.dataset.get_y(self.t - 2)[2:])):
                #print "bootstrapped"
                self.bootstrapped_s = True