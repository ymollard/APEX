
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
        s = self.model.imodel.fmodel.dataset.get_y(idx)
        _, idxs = self.model.imodel.fmodel.dataset.nn_y(s, k=1000)
        # Find nearest s that was not a demo
        for idx in idxs:
            m = self.model.imodel.fmodel.dataset.get_x(idx)
            if np.linalg.norm(m) > 0:
                break
        r = m
        r[self.sigma_expl > 0] = np.random.normal(r[self.sigma_expl > 0], self.sigma_expl[self.sigma_expl > 0])
        res = bounds_min_max(r, self.m_mins, self.m_maxs)
        return res
    