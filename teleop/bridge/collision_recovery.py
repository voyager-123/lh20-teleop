"""Allow existing CAD contacts to separate; never introduce/deepen contacts."""
import math


class RecoveryEnvelope:
    def __init__(self, depths, tolerance=1e-5):
        if not depths or any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in depths.values()):
            raise ValueError('Recovery requires known nonnegative penetration metrics')
        self.initial=dict(depths)
        self.best=dict(depths)
        self.tolerance=tolerance

    def advance(self, depths):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in depths.values()):
            return False,'unknown_contact_depth'
        if set(depths)-set(self.initial):return False,'new_collision_pair'
        for pair,depth in depths.items():
            if depth>self.best[pair]+self.tolerance:return False,'existing_collision_deepened'
        for pair in self.best:self.best[pair]=min(self.best[pair],depths.get(pair,0.))
        return True,'separating_or_nonworsening_existing_contact'
