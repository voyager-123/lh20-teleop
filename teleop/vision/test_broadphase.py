"""Compare conservative pruning with exhaustive real-mesh queries; no devices."""
import unittest
import numpy as np
from factory_hand_audit import FactoryAudit


class BroadphaseTests(unittest.TestCase):
    def test_both_sides_and_thumb_adapters_match_exhaustive_contacts_and_depths(self):
        rng=np.random.default_rng(20261105)
        for hand in ('left','right'):
            for axes in ('native','swapped'):
                fast=FactoryAudit(axes,hand)
                full=FactoryAudit(axes,hand,use_broadphase=False)
                for step in range(16):
                    named={n:float(rng.uniform(*bounds)) for n,bounds in fast.mapping.limits.items()}
                    if step==0:named={n:0. for n in named}
                    q=fast.checker.expand(named)
                    for detailed in (False,True):
                        with self.subTest(hand=hand,axes=axes,step=step,detailed=detailed):
                            a,b=fast.snapshot(q,detailed),full.snapshot(q,detailed)
                            self.assertEqual(a['limits'],b['limits'])
                            self.assertEqual(a['collisions'],b['collisions'])
                            self.assertEqual(a['complete'],b['complete'])
                            self.assertEqual(set(a['depths']),set(b['depths']))
                            for pair in a['depths']:self.assertAlmostEqual(a['depths'][pair],b['depths'][pair],places=12)

    def test_margin_or_invalid_bounds_fall_back_to_all_pairs(self):
        audit=FactoryAudit(hand='right')
        q=audit.checker.expand({n:0. for n in audit.mapping.joints})
        audit.snapshot(q)
        total=len(audit.checker.geometry.collisionPairs)
        self.assertLess(len(audit.overlapping_pairs()),total)
        audit.checker.gdata.collisionRequests[0].security_margin=.001
        self.assertEqual(len(audit.overlapping_pairs()),total)
        audit.checker.gdata.collisionRequests[0].security_margin=0
        audit.boxes_valid=False
        self.assertEqual(len(audit.overlapping_pairs()),total)

    def test_moving_and_existing_contact_paths_match_exhaustive_results(self):
        raw=[255,202,200,198,208,136,79,131,178,224,238,0,0,0,0,92,254,255,255,255]
        audits=[FactoryAudit(hand='right',use_broadphase=x) for x in (False,True)]
        for slot,delta in ((9,-3),(5,5),(1,5),(10,-5)):
            target=raw.copy();target[slot]+=delta
            self.assertEqual(audits[0].check(raw,target),audits[1].check(raw,target))
        self.assertEqual(audits[0].check(raw,raw),audits[1].check(raw,raw))


if __name__=='__main__':unittest.main()
