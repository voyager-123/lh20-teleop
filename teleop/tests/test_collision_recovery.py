"""The recovery exception cannot create or deepen an existing CAD contact."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bridge'))
from collision_recovery import RecoveryEnvelope


class RecoveryPolicyTests(unittest.TestCase):
    def test_separation_can_pass_through_smaller_existing_overlap_until_clear(self):
        r=RecoveryEnvelope({'thumb-index':.004})
        for d in [.003,.002,.001]:self.assertTrue(r.advance({'thumb-index':d})[0])
        self.assertTrue(r.advance({})[0])

    def test_deeper_overlap_is_still_blocked(self):
        r=RecoveryEnvelope({'thumb-index':.004})
        self.assertFalse(r.advance({'thumb-index':.0041})[0])

    def test_new_collision_pair_is_still_blocked(self):
        r=RecoveryEnvelope({'thumb-index':.004})
        self.assertFalse(r.advance({'thumb-index':.003,'ring-pinky':.001})[0])

    def test_cleared_pair_cannot_reenter_or_accumulate_tolerance(self):
        r=RecoveryEnvelope({'a':.004});self.assertTrue(r.advance({'a':.003})[0])
        self.assertTrue(r.advance({'a':.003005})[0]);self.assertFalse(r.advance({'a':.003015})[0])
        r=RecoveryEnvelope({'a':.004});self.assertTrue(r.advance({})[0]);self.assertFalse(r.advance({'a':.001})[0])

    def test_unrelated_motion_with_unchanged_existing_contact_can_continue(self):
        r=RecoveryEnvelope({'a':.004});self.assertTrue(r.advance({'a':.004})[0])

    def test_unknown_or_nonfinite_contact_depth_is_rejected(self):
        with self.assertRaises(ValueError):RecoveryEnvelope({'a':float('nan')})
        r=RecoveryEnvelope({'a':.004});self.assertFalse(r.advance({'a':float('nan')})[0])


if __name__=='__main__':unittest.main()
